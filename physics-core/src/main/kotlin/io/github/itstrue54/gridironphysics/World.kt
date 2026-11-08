package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BallRecord
import io.github.itstrue54.gridironphysics.internal.Body
import io.github.itstrue54.gridironphysics.internal.Flight
import io.github.itstrue54.gridironphysics.internal.FlightPoint
import io.github.itstrue54.gridironphysics.internal.GoalPosts
import io.github.itstrue54.gridironphysics.internal.BroadPhase
import io.github.itstrue54.gridironphysics.internal.BruteForceBroadPhase
import io.github.itstrue54.gridironphysics.internal.Collide
import io.github.itstrue54.gridironphysics.internal.Contact
import io.github.itstrue54.gridironphysics.internal.Geom
import io.github.itstrue54.gridironphysics.internal.Joint
import io.github.itstrue54.gridironphysics.internal.Softness
import io.github.itstrue54.gridironphysics.internal.LocomotionController
import io.github.itstrue54.gridironphysics.internal.RayCast
import io.github.itstrue54.gridironphysics.internal.RayResult
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Fnv1a64
import io.github.itstrue54.gridironphysics.math.SplitMix64
import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3

/**
 * The simulation. Advances in fixed 1/60 s ticks; the same seed and the same calls produce the same
 * state on every device. Not thread-safe: call it from one thread.
 */
class World(val config: WorldConfig, val field: FieldConfig, seed: Long) {

    /** Optional hook that sees every state-changing call (for replays). */
    var recorder: CommandRecorder? = null

    /** Ticks completed since creation. */
    var tick: Long = 0L
        private set

    internal val rng = SplitMix64(seed)
    internal val ball = BallRecord()

    init {
        require(config.maxBodies < BodyId.MAX_SLOTS) { "maxBodies must be < ${BodyId.MAX_SLOTS}" }
    }

    /**
     * Slot-indexed storage. Iterating slots in order is iterating bodies in ascending id order. The last slot,
     * [ballSlot], is the loose ball's and holds it only while the ball is [BallState.Loose].
     */
    private val bodies = arrayOfNulls<Body>(config.maxBodies + 1)
    private val generations = IntArray(config.maxBodies + 1)

    /** Live game bodies (the loose ball is not one of them, so it never counts against maxBodies). */
    private var liveCount = 0

    /** One past the highest slot ever used; loops stop here. */
    private var highWater = 0
    private val view = BodyView()

    // ---------------------------------------------------------------- collision state (all preallocated)

    /** Runoff walls: engine-owned static geometry, not bodies (they never appear in [bodyIds]). */
    private val walls: Array<Body> = makeWalls()

    /** Which of [walls] exist: the four runoff walls with [WorldConfig.runoffWalls], the two post bases with [WorldConfig.goalPosts]. */
    private val wallOn = BooleanArray(walls.size) { k -> if (k < ContactView.POST_HOME) config.runoffWalls else config.goalPosts }
    private val broad: BroadPhase = BruteForceBroadPhase()
    private val proxyCap = config.maxBodies + 1 + walls.size
    private val proxyActive = BooleanArray(proxyCap)
    private val proxyMinX = DoubleArray(proxyCap)
    private val proxyMinY = DoubleArray(proxyCap)
    private val proxyMaxX = DoubleArray(proxyCap)
    private val proxyMaxY = DoubleArray(proxyCap)
    private val pairA = IntArray(config.maxContacts * 4)
    private val pairB = IntArray(config.maxContacts * 4)
    private var pairCount = 0
    private var contacts = Array(config.maxContacts) { Contact() }
    private var oldContacts = Array(config.maxContacts) { Contact() }
    private var oldContactCount = 0
    private val contactView = ContactView()

    // Soft contact coefficients (Catto's soft step), fixed because the timestep is fixed.
    private val softBiasRate: Double
    private val softMassScale: Double
    private val softImpulseScale: Double

    init {
        val h = WorldConfig.DT
        val omega = 2.0 * DMath.PI * config.contactHertz
        val a1 = 2.0 * config.contactDampingRatio + h * omega
        val a2 = h * omega * a1
        val a3 = 1.0 / (1.0 + a2)
        softBiasRate = omega / a1
        softMassScale = a2 * a3
        softImpulseScale = a3
    }

    // Sleep islands: union-find over body slots.
    private val islandParent = IntArray(config.maxBodies + 1)
    private val islandMinTime = DoubleArray(config.maxBodies + 1)

    // ---------------------------------------------------------------- the ball (state; behavior below)

    private val ballSlot = config.maxBodies

    /**
     * The ball's id. It names a body (in [bodyIds], [body], queries and contacts) only while the ball is
     * [BallState.Loose]; [deflect] takes it in the air too. It never changes.
     */
    val ballId: BodyId = BodyId.of(ballSlot, 0)

    /** The loose ball's body: engine-owned, built once, put in [ballSlot] when the ball lands and taken out when it leaves the ground. */
    private val ballBody: Body = Body(ballId.value, BodyType.Dynamic, ShapeDef.Circle(config.ball.radius)).also { b ->
        b.mass = config.ball.mass; b.invMass = 1.0 / b.mass
        b.inertia = inertiaAboutOrigin(b.shape, b.mass); b.invInertia = 1.0 / b.inertia
        b.angularDamping = BALL_SPIN_DAMPING
        b.bullet = true
        b.categoryBits = Category.BALL; b.maskBits = Category.ALL
        b.material = Material.BALL; b.baseMaterial = Material.BALL
        b.extent = config.ball.radius
    }
    private val ballView = BallView(ball)
    private val flight = Flight(config.ball, config.gravity)
    private val posts = GoalPosts(field, config.ball.radius)
    private val fp = FlightPoint()

    // ---------------------------------------------------------------- bodies

    /** Creates a body in the lowest free slot. Throws [IllegalStateException] when maxBodies are alive. */
    fun createBody(def: BodyDef): BodyId {
        def.validate()
        check(liveCount < config.maxBodies) { "World is full: ${config.maxBodies} live bodies" }
        var slot = 0
        while (bodies[slot] != null) slot++
        val id = BodyId.of(slot, generations[slot])
        val b = Body(id.value, def.type, def.shape)
        b.x = def.x; b.y = def.y; b.prevX = def.x; b.prevY = def.y
        b.angle = def.angle
        if (def.type != BodyType.Static) {
            b.vx = def.vx; b.vy = def.vy; b.angularVelocity = def.angularVelocity
        }
        b.linearDamping = def.linearDamping; b.angularDamping = def.angularDamping
        b.userData = def.userData; b.bullet = def.bullet
        b.categoryBits = def.categoryBits; b.maskBits = def.maskBits; b.material = def.material
        b.baseMaterial = def.material
        b.extent = shapeExtent(def.shape)
        if (def.type == BodyType.Dynamic) {
            b.mass = def.mass
            b.invMass = 1.0 / def.mass
            b.inertia = inertiaAboutOrigin(def.shape, def.mass)
            b.invInertia = 1.0 / b.inertia
        }
        b.geom.set(b.shape, b.x, b.y, b.angle)
        b.x0 = b.x; b.y0 = b.y; b.angle0 = b.angle
        // A body starts wherever it is: being created on a line or out of bounds is not an event.
        b.lineMask = lineOverlap(b.geom)
        b.outOfBounds = outOfBounds(b.geom)
        bodies[slot] = b
        liveCount++
        if (slot >= highWater) highWater = slot + 1
        recorder?.onCreateBody(tick, id, def)
        return id
    }

    fun destroyBody(id: BodyId) {
        val dead = gameBody(id)
        // Bodies resting against this one must not stay asleep against nothing.
        for (i in 0 until contactCount) {
            val c = contacts[i]
            if (c.a === dead) wake(c.b!!) else if (c.b === dead) wake(c.a!!)
        }
        // Its joints go with it (not a command of their own, and not a break).
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            if (j.a === dead || j.b === dead) removeJoint(j)
        }
        if (carrier === dead) {
            // The ball drops dead where he was.
            carrier = null; forwardProgressX = Double.NaN
            ball.mode = BallRecord.MODE_DEAD; ball.vx = 0.0; ball.vy = 0.0; ball.vz = 0.0
        }
        val slot = id.slot
        bodies[slot] = null
        generations[slot] = (generations[slot] + 1) and BodyId.GEN_MASK
        liveCount--
        recorder?.onDestroyBody(tick, id)
    }

    /** Read-only view of a body. The returned object is reused by the next call. */
    fun body(id: BodyId): BodyView {
        view.b = live(id)
        return view
    }

    /** Number of live bodies, the loose ball included. */
    val bodyCount: Int get() = liveCount + (if (bodies[ballSlot] != null) 1 else 0)

    /**
     * Writes live body ids (as [BodyId.value]) in ascending order into [out] and returns how many were
     * written. If [out] is shorter than [bodyCount], only the first `out.size` ids are written.
     */
    fun bodyIds(out: IntArray): Int {
        var n = 0
        for (i in 0 until highWater) {
            if (n == out.size) break
            val b = bodies[i] ?: continue
            out[n++] = b.id
        }
        return n
    }

    private fun live(id: BodyId): Body {
        val slot = id.slot
        if (id.value == ballId.value && bodies[ballSlot] == null) {
            throw IllegalArgumentException("The ball is a body only while it is Loose; it is ${ballView.state}")
        }
        require(id.value >= 0 && slot < highWater) { "Unknown body id ${id.value}" }
        val b = bodies[slot]
        require(b != null && b.id == id.value) { "Body ${id.value} was destroyed" }
        return b
    }

    /** A live body the game owns: not the loose ball, which belongs to the engine. */
    private fun gameBody(id: BodyId): Body {
        val b = live(id)
        require(b !== ballBody) { "Body ${id.value} is the ball, which belongs to the engine: use launchBall, deflect and setBallCarrier" }
        return b
    }

    private fun wake(b: Body) {
        if (b.type == BodyType.Dynamic && !b.awake) {
            b.awake = true
            b.sleepTime = 0.0
        }
    }

    // ---------------------------------------------------------------- locomotion

    /** The playing surface; it scales every athlete's traction. */
    var surface: SurfaceConfig = SurfaceConfig.ARTIFICIAL_TURF
        private set

    fun setSurface(surface: SurfaceConfig) {
        this.surface = surface
        recorder?.onSetSurface(tick, surface)
    }

    /** Gives a dynamic body an athlete's limits, which it needs before [setIntent]. */
    fun setAthlete(id: BodyId, params: AthleteParams) {
        val b = gameBody(id)
        require(b.type == BodyType.Dynamic) { "Only dynamic bodies can have an athlete: body ${id.value} is ${b.type}" }
        val c = b.loco
        if (c == null) b.loco = LocomotionController(params) else c.params = params
        recorder?.onSetAthlete(tick, id, params)
    }

    /**
     * Sets what the player tries to do from the next tick on, and switches stance when [LocomotionIntent.stance]
     * is not null. The values are copied, so the game can reuse [intent]. Wakes the body.
     */
    fun setIntent(id: BodyId, intent: LocomotionIntent) {
        val b = gameBody(id)
        val c = checkNotNull(b.loco) { "Body ${id.value} has no athlete: call setAthlete first" }
        intent.validate()
        val stance = intent.stance
        if (stance != null) checkStanceBody(b)
        c.intent.copyFrom(intent)
        if (stance != null) applyStance(b, stance)
        wake(b)
        recorder?.onSetIntent(tick, id, intent)
    }

    /**
     * Switches stance and swaps the collider in place: Upright is the circle the body was created with; Diving
     * and Down are a capsule along the body angle (Down also uses [Material.DOWNED_PLAYER]). Mass stays; the
     * rotational inertia follows the shape. Only dynamic bodies created with a circle have stances. Wakes the body.
     * A Diving or Down body with an athlete ([setAthlete]) slides, braked by [WorldConfig.slideFriction], and
     * ignores its intent until it is Upright again; one without an athlete just coasts under its damping.
     */
    fun setStance(id: BodyId, stance: Stance) {
        val b = gameBody(id)
        checkStanceBody(b)
        applyStance(b, stance)
        wake(b)
        recorder?.onSetStance(tick, id, stance)
    }

    private fun checkStanceBody(b: Body) {
        require(b.type == BodyType.Dynamic) { "Only dynamic bodies have a stance: body ${b.id} is ${b.type}" }
        require(b.uprightShape is ShapeDef.Circle) { "Stance needs a body created with a Circle (its Upright collider): body ${b.id} is ${b.uprightShape}" }
    }

    private fun applyStance(b: Body, stance: Stance) {
        val up = b.uprightShape as ShapeDef.Circle
        val shape: ShapeDef = if (stance == Stance.Upright) {
            up
        } else {
            b.lyingShape ?: ShapeDef.Capsule(LYING_HALF_LENGTH * up.radius, LYING_RADIUS * up.radius, up.sensor).also { b.lyingShape = it }
        }
        if (stance == Stance.Down && b.stance != Stance.Down) {
            // Stances change between steps, so the event carries the current tick.
            val e = eventReader.add(EventReader.T_DOWN, tick, b.id)
            if (b === carrier) {
                eventReader.setInt(e, EventReader.I_CARRIER, 1)
                eventReader.setDouble(e, EventReader.D_VALUE, forwardProgressX)
            }
        }
        b.stance = stance
        b.material = if (stance == Stance.Down) Material.DOWNED_PLAYER else b.baseMaterial
        if (shape !== b.shape) {
            b.shape = shape
            b.extent = shapeExtent(shape)
            b.inertia = inertiaAboutOrigin(shape, b.mass)
            b.invInertia = 1.0 / b.inertia
            b.geom.set(shape, b.x, b.y, b.angle)
        }
    }

    // ---------------------------------------------------------------- contacts

    /**
     * Number of contacts found by the last step, sorted by (lower body id, higher body id), with a body's
     * runoff-wall contacts after its body contacts. Destroying a body leaves its contacts listed until the next step.
     */
    var contactCount: Int = 0
        private set

    /** Read-only view of contact [index] from the last step. The returned object is reused by the next call. */
    fun contact(index: Int): ContactView {
        require(index in 0 until contactCount) { "contact $index out of range 0 until $contactCount" }
        contactView.c = contacts[index]
        return contactView
    }

    // ---------------------------------------------------------------- joints

    // Slot-indexed like bodies: iterating slots in order is iterating joints in ascending id order, which is the
    // order the solver runs them in.
    private val joints = arrayOfNulls<Joint>(config.maxJoints)
    private val jointGenerations = IntArray(config.maxJoints)
    private var jointLive = 0
    private var jointHighWater = 0
    private val jointView = JointView()

    /** Softness of rigid joint rows while solving velocities; the relax pass runs them fully rigid. */
    private val rigidJoint = Softness.of(config.jointHertz, config.jointDampingRatio)

    /**
     * A tackler wraps up a carrier: a soft, pull-only rope between their centers that breaks when its pull reaches
     * [GrappleDef.breakForce] ([EventType.GrappleBroken]), plus drag that slows the carrier. Grapples on one carrier
     * stack. The tackler may be any body type (a static one is a tackler with planted feet); the carrier must be
     * dynamic. Wakes both. The engine never decides the tackle: the game sets the carrier [Stance.Down].
     */
    fun createGrapple(tackler: BodyId, carrier: BodyId, def: GrappleDef): JointId {
        val t = live(tackler); val c = live(carrier)
        require(c.type == BodyType.Dynamic) { "The carrier must be dynamic: body ${carrier.value} is ${c.type}" }
        val j = newJoint(JointType.Grapple, t, c, def.length, Softness.of(def.hertz, def.dampingRatio))
        j.breakForce = def.breakForce
        j.dragForce = def.dragForce
        val id = JointId(j.id)
        recorder?.onCreateGrapple(tick, id, tackler, carrier, def)
        return id
    }

    /**
     * Engages a block between [blocker] and [defender]; see [BlockDef]. Both bodies stay awake while engaged.
     * Several blocks on one defender (a double team) each push and shed on their own.
     */
    fun createBlock(blocker: BodyId, defender: BodyId, def: BlockDef): JointId {
        val a = live(blocker); val b = live(defender)
        val j = newJoint(JointType.Block, a, b, def.length, Softness.of(def.hertz, def.dampingRatio))
        j.pushForceA = def.blockerPushForce
        j.pushForceB = def.defenderPushForce
        j.shedThreshold = def.shedThreshold
        j.driveSpeed = def.driveSpeed
        val id = JointId(j.id)
        recorder?.onCreateBlock(tick, id, blocker, defender, def)
        return id
    }

    /** Holds the centers of [a] and [b] at a distance, rigidly or as a spring; see [DistanceDef]. Wakes both. */
    fun createDistance(a: BodyId, b: BodyId, def: DistanceDef): JointId {
        val ba = live(a); val bb = live(b)
        val j = newJoint(JointType.Distance, ba, bb, def.length, if (def.hertz == 0.0) null else Softness.of(def.hertz, def.dampingRatio))
        val id = JointId(j.id)
        recorder?.onCreateDistance(tick, id, a, b, def)
        return id
    }

    /** Glues [b] to [a] in their current relative position and angle; see [WeldDef]. Wakes both. */
    fun createWeld(a: BodyId, b: BodyId, def: WeldDef): JointId {
        val ba = live(a); val bb = live(b)
        val j = newJoint(JointType.Weld, ba, bb, null, if (def.hertz == 0.0) null else Softness.of(def.hertz, def.dampingRatio))
        val c = DMath.cos(ba.angle); val s = DMath.sin(ba.angle)
        val dx = bb.x - ba.x; val dy = bb.y - ba.y
        j.localX = c * dx + s * dy
        j.localY = -s * dx + c * dy
        j.refAngle = bb.angle - ba.angle
        val id = JointId(j.id)
        recorder?.onCreateWeld(tick, id, a, b, def)
        return id
    }

    /** Removes a joint and wakes both bodies. Throws [IllegalArgumentException] for a joint that is gone (destroyed, broken or shed). */
    fun destroyJoint(id: JointId) {
        removeJoint(liveJoint(id))
        recorder?.onDestroyJoint(tick, id)
    }

    /** True while the joint exists: false once it is destroyed, broken or shed, or one of its bodies was destroyed. */
    fun hasJoint(id: JointId): Boolean {
        if (id.value < 0) return false
        val slot = id.slot
        if (slot >= jointHighWater) return false
        val j = joints[slot]
        return j != null && j.id == id.value
    }

    /** Read-only view of a joint. The returned object is reused by the next call. */
    fun joint(id: JointId): JointView {
        jointView.j = liveJoint(id)
        return jointView
    }

    /** Number of live joints. */
    val jointCount: Int get() = jointLive

    /** Writes live joint ids ([JointId.value]) in ascending order into [out] and returns how many (at most `out.size`). */
    fun jointIds(out: IntArray): Int {
        var n = 0
        for (k in 0 until jointHighWater) {
            if (n == out.size) break
            val j = joints[k] ?: continue
            out[n++] = j.id
        }
        return n
    }

    private fun liveJoint(id: JointId): Joint {
        require(hasJoint(id)) { "Joint ${id.value} does not exist (destroyed, broken, shed or never created)" }
        return joints[id.slot]!!
    }

    private fun newJoint(type: JointType, a: Body, b: Body, length: Double?, soft: Softness?): Joint {
        require(a !== b) { "A joint needs two different bodies: ${a.id}" }
        require(a !== ballBody && b !== ballBody) { "The ball cannot be jointed" }
        require(a.type == BodyType.Dynamic || b.type == BodyType.Dynamic) { "A joint needs a dynamic body: ${a.id} is ${a.type}, ${b.id} is ${b.type}" }
        check(jointLive < config.maxJoints) { "Joint pool is full: ${config.maxJoints} live joints" }
        var slot = 0
        while (joints[slot] != null) slot++
        val id = JointId.of(slot, jointGenerations[slot])
        val rest = length ?: DMath.hypot2(b.x - a.x, b.y - a.y)
        // A centre-to-centre joint of length 0 has no direction to act along.
        require(type == JointType.Weld || rest > Joint.MIN_LENGTH) { "Bodies ${a.id} and ${b.id} share a center: pass a length in the def" }
        val j = Joint(id.value, type, a, b, rest, soft)
        joints[slot] = j
        jointLive++
        if (slot >= jointHighWater) jointHighWater = slot + 1
        wake(a); wake(b)
        return j
    }

    private fun removeJoint(j: Joint) {
        val slot = JointId(j.id).slot
        joints[slot] = null
        jointGenerations[slot] = (jointGenerations[slot] + 1) and BodyId.GEN_MASK
        jointLive--
        wake(j.a); wake(j.b)
    }

    // ---------------------------------------------------------------- ball carrier and forward progress

    private var carrier: Body? = null

    /** +1 when the offense attacks toward +x, −1 toward −x. Defaults to +1. */
    var attackDirection: Int = 1
        private set

    /**
     * The ball carrier's farthest x toward [attackDirection] while he was [Stance.Upright] with the ball, as of
     * the end of the last step (or the moment he got the ball). NaN when there is no carrier. The [EventType.Down]
     * event reports it; the game spots the ball there.
     */
    var forwardProgressX: Double = Double.NaN
        private set

    /** The body holding the ball, or null. */
    val ballCarrier: BodyId? get() = carrier?.let { BodyId(it.id) }

    /**
     * Gives the ball to [id] (a dynamic body), or takes it away with null. Possession is the game's call: giving the
     * ball ends its flight (a catch) or picks it up off the ground, and the ball is [BallState.Carried] from then on.
     * Taking it from its carrier with null makes it [BallState.Dead] where it is. Tracking restarts from the new
     * carrier's current x.
     */
    fun setBallCarrier(id: BodyId?) {
        if (id == null) {
            carrier = null
            forwardProgressX = Double.NaN
            if (ball.mode == BallRecord.MODE_CARRIED) { ball.mode = BallRecord.MODE_DEAD; ball.vx = 0.0; ball.vy = 0.0; ball.vz = 0.0 }
        } else {
            val b = gameBody(id)
            require(b.type == BodyType.Dynamic) { "The ball carrier must be dynamic: body ${id.value} is ${b.type}" }
            if (ball.mode == BallRecord.MODE_LOOSE) removeBallBody()
            carrier = b
            forwardProgressX = b.x
            ball.mode = BallRecord.MODE_CARRIED
            carryBall()
            ball.prevX = ball.x; ball.prevY = ball.y; ball.prevZ = ball.z
        }
        recorder?.onSetBallCarrier(tick, id)
    }

    /** Sets which way the offense attacks (+1 or −1). Tracking restarts from the carrier's current x. */
    fun setAttackDirection(direction: Int) {
        require(direction == 1 || direction == -1) { "attack direction must be +1 or -1: $direction" }
        attackDirection = direction
        val c = carrier
        if (c != null) forwardProgressX = c.x
        recorder?.onSetAttackDirection(tick, direction)
    }

    // ---------------------------------------------------------------- events

    private val eventReader = EventReader(config.eventCapacity)

    /**
     * The event stream: contacts, hits, trips, slips and field-line crossings. Drain it after each [step] or
     * [advance]; unread events stay until the buffer is full. Always returns the same reader. See [EventReader].
     */
    fun events(): EventReader = eventReader

    // ---------------------------------------------------------------- queries

    // Queries read the world as of the end of the last step (shapes are refreshed there), never change it and
    // never allocate. They see bodies only: runoff walls are not reported. Ties go to the lower body id.

    private val ray = RayResult()
    private val nearDist = DoubleArray(config.maxBodies + 1)

    private fun passes(b: Body, f: QueryFilter): Boolean =
        (b.categoryBits and f.maskBits) != 0 && b.id != f.ignore && (f.includeSensors || !b.sensor)

    /** The first body the segment [from]→[to] hits, written to [out]. Returns false when it hits nothing. */
    fun raycast(from: Vec2, to: Vec2, filter: QueryFilter, out: RayHit): Boolean = cast(from, to, 0.0, filter, out)

    /**
     * Sweeps a circle of [radius] from [from] to [to] and reports the first body it touches; [RayHit.pointX] is the
     * circle's center at that moment. A circle that starts overlapping a body hits it at fraction 0.
     */
    fun circleCast(from: Vec2, to: Vec2, radius: Double, filter: QueryFilter, out: RayHit): Boolean {
        require(radius.isFinite() && radius >= 0.0) { "radius must be finite and >= 0: $radius" }
        return cast(from, to, radius, filter, out)
    }

    private fun cast(from: Vec2, to: Vec2, radius: Double, filter: QueryFilter, out: RayHit): Boolean {
        val ox = from.x; val oy = from.y
        val dx = to.x - ox; val dy = to.y - oy
        require(ox.isFinite() && oy.isFinite() && dx.isFinite() && dy.isFinite()) { "cast points must be finite" }
        require(dx != 0.0 || dy != 0.0) { "cast needs from != to" }
        val rMinX = DMath.min(ox, to.x) - radius; val rMaxX = DMath.max(ox, to.x) + radius
        val rMinY = DMath.min(oy, to.y) - radius; val rMaxY = DMath.max(oy, to.y) + radius
        var best = Double.MAX_VALUE
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (!passes(b, filter)) continue
            val g = b.geom
            if (g.minX > rMaxX || g.maxX < rMinX || g.minY > rMaxY || g.maxY < rMinY) continue
            if (!RayCast.cast(g, radius, ox, oy, dx, dy, ray) || ray.t >= best) continue
            best = ray.t
            out.body = BodyId(b.id); out.fraction = ray.t
            out.pointX = ox + ray.t * dx; out.pointY = oy + ray.t * dy
            out.normalX = ray.nx; out.normalY = ray.ny
        }
        return best != Double.MAX_VALUE
    }

    /**
     * Writes the ids ([BodyId.value]) of bodies whose shape bounds overlap the box, in ascending id order, and
     * returns how many. Stops when [out] is full.
     */
    fun queryAabb(minX: Double, minY: Double, maxX: Double, maxY: Double, filter: QueryFilter, out: IntArray): Int {
        require(minX <= maxX && minY <= maxY) { "empty box: ($minX, $minY)-($maxX, $maxY)" }
        var n = 0
        for (i in 0 until highWater) {
            if (n == out.size) break
            val b = bodies[i] ?: continue
            if (!passes(b, filter)) continue
            val g = b.geom
            if (g.minX > maxX || g.maxX < minX || g.minY > maxY || g.maxY < minY) continue
            out[n++] = b.id
        }
        return n
    }

    /**
     * Writes the ids ([BodyId.value]) of up to [count] bodies nearest [p] (by center distance) into [out], nearest
     * first, and returns how many. [count] must fit in [out].
     */
    fun nearestBodies(p: Vec2, count: Int, filter: QueryFilter, out: IntArray): Int {
        require(count in 0..out.size) { "count must be in 0..${out.size}: $count" }
        if (count == 0) return 0
        var n = 0
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (!passes(b, filter)) continue
            val dx = b.x - p.x; val dy = b.y - p.y
            val d2 = dx * dx + dy * dy
            if (n == count && d2 >= nearDist[n - 1]) continue
            // Insertion sort. A later (higher) id goes after an equal distance, so ties stay in id order.
            var k = if (n == count) n - 1 else n++
            while (k > 0 && nearDist[k - 1] > d2) { nearDist[k] = nearDist[k - 1]; out[k] = out[k - 1]; k-- }
            nearDist[k] = d2; out[k] = b.id
        }
        return n
    }

    // ---------------------------------------------------------------- frame accumulator

    private var accumulator = 0.0

    /** Leftover fraction of a tick after the last [advance], in [0, 1). Use it to interpolate drawing. */
    var alpha: Double = 0.0
        private set

    /**
     * Feeds one rendered frame's real time. Runs 0 to [WorldConfig.maxTicksPerFrame] ticks, drops any
     * time beyond that (so a long pause never causes a burst of catch-up ticks), and returns [alpha].
     * Frame timing is not part of the deterministic simulation; replays record ticks, not frame times.
     */
    fun advance(frameSeconds: Double): Double {
        require(frameSeconds.isFinite()) { "frameSeconds must be finite: $frameSeconds" }
        val dt = WorldConfig.DT
        if (frameSeconds > 0.0) accumulator += frameSeconds
        var ticks = 0
        while (ticks < config.maxTicksPerFrame && accumulator >= dt - ADVANCE_EPS) {
            step()
            accumulator -= dt
            ticks++
        }
        // Keep the leftover strictly below one tick's threshold so dropped time never resurfaces.
        val maxLeft = dt - 2 * ADVANCE_EPS
        accumulator = if (accumulator < 0.0) 0.0 else if (accumulator > maxLeft) maxLeft else accumulator
        alpha = accumulator / dt
        return alpha
    }

    // ---------------------------------------------------------------- step

    /** Advances exactly one 1/60 s tick, running the spec's ten phases in order. */
    fun step() {
        wakeJointBodies()
        applyIntents()
        applyJointForces()
        integrateVelocities()
        broadPhase()
        narrowPhase()
        solveVelocities()
        integratePositions()
        relax()
        continuousCollision()
        breakJoints()
        ballFlight()
        emitEvents()
        bookkeeping()
    }

    /** Each awake player's locomotion controller turns its intent into a force, clamped by traction. */
    private fun applyIntents() {
        val surfaceGrip = surface.tractionMultiplier * config.gravity
        val slideGrip = config.slideFriction * surfaceGrip
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            val c = b.loco ?: continue
            if (!b.awake) continue
            c.drive(b, c.params.traction * surfaceGrip, slideGrip)
            if (c.slip > 0.0) {
                val e = eventReader.add(EventReader.T_SLIP, tick + 1, b.id)
                eventReader.setDouble(e, EventReader.D_VALUE, c.slip)
            }
        }
    }

    /** A joint with a moving body wakes its other body; an engaged block keeps both awake. Drag starts each tick at zero. */
    private fun wakeJointBodies() {
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            j.beginTick()
            if (j.type == JointType.Block || j.a.active || j.b.active) { wake(j.a); wake(j.b) }
        }
    }

    /** Block pushes (ground reaction through each player's feet), in joint id order, on top of locomotion. */
    private fun applyJointForces() {
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            if (j.type == JointType.Block) j.applyPushes()
        }
    }

    private fun integrateVelocities() {
        val dt = WorldConfig.DT
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic || !b.awake) continue
            b.vx += b.fx * b.invMass * dt
            b.vy += b.fy * b.invMass * dt
            b.fx = 0.0; b.fy = 0.0
            if (b === ballBody) rollBall(b)
            // Damping uses the Box2D form, stable for any damping value.
            b.vx *= 1.0 / (1.0 + dt * b.linearDamping)
            b.vy *= 1.0 / (1.0 + dt * b.linearDamping)
            b.angularVelocity *= 1.0 / (1.0 + dt * b.angularDamping)
        }
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            b.preVx = b.vx; b.preVy = b.vy
        }
    }

    private fun proxyBody(p: Int): Body = if (p < highWater) bodies[p]!! else walls[p - highWater]

    /**
     * Builds fat AABBs, then finds candidate pairs in (lower id, higher id) order. World-space shapes are already
     * current: the end of the last step (or createBody / setStance) refreshed them.
     */
    private fun broadPhase() {
        val dt = WorldConfig.DT
        val n = highWater + walls.size
        for (p in 0 until n) {
            val b = if (p < highWater) bodies[p] else if (wallOn[p - highWater]) walls[p - highWater] else null
            if (b == null) { proxyActive[p] = false; continue }
            proxyActive[p] = true
            b.x0 = b.x; b.y0 = b.y; b.angle0 = b.angle
            b.speedBound = if (b.active) DMath.hypot2(b.vx, b.vy) + DMath.abs(b.angularVelocity) * b.extent else 0.0
            val fat = 0.5 * Collide.SPECULATIVE + b.speedBound * dt
            val g = b.geom
            proxyMinX[p] = g.minX - fat; proxyMinY[p] = g.minY - fat
            proxyMaxX[p] = g.maxX + fat; proxyMaxY[p] = g.maxY + fat
        }
        pairCount = broad.findPairs(n, proxyActive, proxyMinX, proxyMinY, proxyMaxX, proxyMaxY, pairA, pairB)
    }

    private fun shouldCollide(a: Body, b: Body): Boolean {
        if (a.type != BodyType.Dynamic && b.type != BodyType.Dynamic) return false
        if (!a.active && !b.active) return false
        return (a.categoryBits and b.maskBits) != 0 && (b.categoryBits and a.maskBits) != 0
    }

    private fun sortKey(b: Body): Long = if (b.id >= 0) b.id.toLong() else WALL_KEY_BASE + (-1 - b.id)

    /**
     * Builds manifolds for candidate pairs. The speculative margin covers how far the pair can close in one
     * tick, so fast players meet surface to surface instead of overlapping first. Impulses carry over from
     * the last tick by matching feature ids (warm start).
     */
    private fun narrowPhase() {
        val swap = oldContacts; oldContacts = contacts; contacts = swap
        oldContactCount = contactCount
        var count = 0
        var old = 0
        val mats = config.materials
        for (k in 0 until pairCount) {
            if (count == contacts.size) break
            val a = proxyBody(pairA[k])
            val b = proxyBody(pairB[k])
            if (!shouldCollide(a, b)) continue
            val c = contacts[count]
            val margin = Collide.SPECULATIVE + WorldConfig.DT * (a.speedBound + b.speedBound)
            Collide.collide(a.geom, b.geom, c.m, margin)
            if (c.m.count == 0) continue
            c.a = a; c.b = b
            c.idA = a.id; c.idB = b.id
            c.key = (sortKey(a) shl 32) or sortKey(b)
            c.wall = if (b.id < 0) -1 - b.id else ContactView.NO_WALL
            c.sensor = a.sensor || b.sensor ||
                ((a.categoryBits and Category.BALL) != 0 && (b.categoryBits and Category.PLAYERS) != 0) ||
                ((b.categoryBits and Category.BALL) != 0 && (a.categoryBits and Category.PLAYERS) != 0)
            c.friction = mats.friction(a.material, b.material)
            c.restitution = mats.restitution(a.material, b.material)
            for (i in 0 until c.m.count) { c.normalImpulse[i] = 0.0; c.tangentImpulse[i] = 0.0 }
            // Warm start: both lists are sorted by key, so one forward pass matches them.
            while (old < oldContactCount && oldContacts[old].key < c.key) old++
            if (old < oldContactCount && oldContacts[old].key == c.key) {
                val o = oldContacts[old]
                for (i in 0 until c.m.count) {
                    for (j in 0 until o.m.count) {
                        if (o.m.id[j] == c.m.id[i]) {
                            c.normalImpulse[i] = o.normalImpulse[j]
                            c.tangentImpulse[i] = o.tangentImpulse[j]
                            break
                        }
                    }
                }
            }
            if (!c.sensor) {
                // A contact with something that moves wakes a sleeper.
                if (a.active) wake(b)
                if (b.active) wake(a)
            }
            count++
        }
        contactCount = count
        // The old pool no longer describes the world; drop its body references.
        for (i in 0 until oldContactCount) { oldContacts[i].a = null; oldContacts[i].b = null }
    }

    /**
     * Prepares every joint and contact, applies the warm-start impulses, then runs the velocity iterations with soft
     * bias: joints first (in id order), then contacts.
     */
    private fun solveVelocities() {
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            j.prepare()
            j.warmStart()
        }
        for (i in 0 until contactCount) {
            val c = contacts[i]
            val a = c.a!!; val b = c.b!!
            val mA = a.invMass; val iA = a.invInertia
            val mB = b.invMass; val iB = b.invInertia
            c.solve = !c.sensor && (mA + mB) > 0.0
            if (!c.solve) continue
            val m = c.m
            val nx = m.nx; val ny = m.ny
            val tx = ny; val ty = -nx
            for (j in 0 until m.count) {
                val rAx = m.px[j] - a.x; val rAy = m.py[j] - a.y
                val rBx = m.px[j] - b.x; val rBy = m.py[j] - b.y
                c.rAx[j] = rAx; c.rAy[j] = rAy; c.rBx[j] = rBx; c.rBy[j] = rBy
                val rnA = rAx * ny - rAy * nx
                val rnB = rBx * ny - rBy * nx
                c.normalMass[j] = 1.0 / (mA + mB + iA * rnA * rnA + iB * rnB * rnB)
                val rtA = rAx * ty - rAy * tx
                val rtB = rBx * ty - rBy * tx
                c.tangentMass[j] = 1.0 / (mA + mB + iA * rtA * rtA + iB * rtB * rtB)
                val dvx = b.vx - b.angularVelocity * rBy - a.vx + a.angularVelocity * rAy
                val dvy = b.vy + b.angularVelocity * rBx - a.vy - a.angularVelocity * rAx
                c.relativeVelocity[j] = dvx * nx + dvy * ny
                c.maxNormalImpulse[j] = 0.0
            }
            for (j in 0 until m.count) {
                val px = c.normalImpulse[j] * nx + c.tangentImpulse[j] * tx
                val py = c.normalImpulse[j] * ny + c.tangentImpulse[j] * ty
                applyImpulse(a, b, c.rAx[j], c.rAy[j], c.rBx[j], c.rBy[j], px, py)
            }
        }
        for (it in 0 until config.velocityIterations) {
            for (k in 0 until jointHighWater) joints[k]?.solveVelocity(rigidJoint, false)
            for (i in 0 until contactCount) if (contacts[i].solve) solveContact(contacts[i], true)
        }
    }

    private fun applyImpulse(a: Body, b: Body, rAx: Double, rAy: Double, rBx: Double, rBy: Double, px: Double, py: Double) {
        a.vx -= a.invMass * px; a.vy -= a.invMass * py
        a.angularVelocity -= a.invInertia * (rAx * py - rAy * px)
        b.vx += b.invMass * px; b.vy += b.invMass * py
        b.angularVelocity += b.invInertia * (rBx * py - rBy * px)
    }

    /**
     * One sequential-impulse pass over a contact. With [useBias], penetration is pushed out softly (spring-damper,
     * capped at maxPushoutSpeed); without it (relax), the contact is rigid and removes that push-out velocity.
     * Speculative points (positive separation) let the bodies close exactly the remaining gap.
     */
    private fun solveContact(c: Contact, useBias: Boolean) {
        val a = c.a!!; val b = c.b!!
        val m = c.m
        val nx = m.nx; val ny = m.ny
        val tx = ny; val ty = -nx
        val inv = 1.0 / WorldConfig.DT
        val dpx = (b.x - b.x0) - (a.x - a.x0)
        val dpy = (b.y - b.y0) - (a.y - a.y0)
        val daA = a.angle - a.angle0
        val daB = b.angle - b.angle0
        for (j in 0 until m.count) {
            val rAx = c.rAx[j]; val rAy = c.rAy[j]; val rBx = c.rBx[j]; val rBy = c.rBy[j]
            // Current separation: manifold value plus how far the anchors moved apart along the normal this step
            // (rotation linearized: a small turn dθ moves an anchor r by dθ × r).
            val dx = dpx - daB * rBy + daA * rAy
            val dy = dpy + daB * rBx - daA * rAx
            val s = m.sep[j] + dx * nx + dy * ny
            val bias: Double
            var massScale = 1.0
            var impulseScale = 0.0
            if (s > 0.0) {
                bias = s * inv
            } else if (useBias) {
                bias = DMath.max(softBiasRate * s, -config.maxPushoutSpeed)
                massScale = softMassScale
                impulseScale = softImpulseScale
            } else {
                bias = 0.0
            }
            val dvx = b.vx - b.angularVelocity * rBy - a.vx + a.angularVelocity * rAy
            val dvy = b.vy + b.angularVelocity * rBx - a.vy - a.angularVelocity * rAx
            val vn = dvx * nx + dvy * ny
            val old = c.normalImpulse[j]
            var impulse = -c.normalMass[j] * massScale * (vn + bias) - impulseScale * old
            val acc = DMath.max(old + impulse, 0.0)
            impulse = acc - old
            c.normalImpulse[j] = acc
            if (impulse > c.maxNormalImpulse[j]) c.maxNormalImpulse[j] = impulse
            applyImpulse(a, b, rAx, rAy, rBx, rBy, impulse * nx, impulse * ny)
        }
        val mu = c.friction
        if (mu == 0.0) return
        for (j in 0 until m.count) {
            val rAx = c.rAx[j]; val rAy = c.rAy[j]; val rBx = c.rBx[j]; val rBy = c.rBy[j]
            val dvx = b.vx - b.angularVelocity * rBy - a.vx + a.angularVelocity * rAy
            val dvy = b.vy + b.angularVelocity * rBx - a.vy - a.angularVelocity * rAx
            val vt = dvx * tx + dvy * ty
            val old = c.tangentImpulse[j]
            val limit = mu * c.normalImpulse[j]
            val acc = DMath.clamp(old - c.tangentMass[j] * vt, -limit, limit)
            c.tangentImpulse[j] = acc
            val impulse = acc - old
            applyImpulse(a, b, rAx, rAy, rBx, rBy, impulse * tx, impulse * ty)
        }
    }

    private fun integratePositions() {
        val dt = WorldConfig.DT
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            b.prevX = b.x
            b.prevY = b.y
            if (b.type == BodyType.Static || !b.awake) continue
            b.x += b.vx * dt
            b.y += b.vy * dt
            b.angle += b.angularVelocity * dt
        }
    }

    /** Rigid (unbiased) iterations that remove push-out velocity, then restitution against pre-solve closing speed. */
    private fun relax() {
        for (k in 0 until jointHighWater) joints[k]?.prepare()
        for (it in 0 until config.relaxIterations) {
            for (k in 0 until jointHighWater) joints[k]?.solveVelocity(rigidJoint, true)
            for (i in 0 until contactCount) if (contacts[i].solve) solveContact(contacts[i], false)
        }
        val threshold = config.restitutionThreshold
        for (i in 0 until contactCount) {
            val c = contacts[i]
            if (!c.solve || c.restitution == 0.0) continue
            val a = c.a!!; val b = c.b!!
            val m = c.m
            val nx = m.nx; val ny = m.ny
            for (j in 0 until m.count) {
                if (c.relativeVelocity[j] > -threshold || c.maxNormalImpulse[j] == 0.0) continue
                val rAx = c.rAx[j]; val rAy = c.rAy[j]; val rBx = c.rBx[j]; val rBy = c.rBy[j]
                val dvx = b.vx - b.angularVelocity * rBy - a.vx + a.angularVelocity * rAy
                val dvy = b.vy + b.angularVelocity * rBx - a.vy - a.angularVelocity * rAx
                val vn = dvx * nx + dvy * ny
                val old = c.normalImpulse[j]
                var impulse = -c.normalMass[j] * (vn + c.restitution * c.relativeVelocity[j])
                val acc = DMath.max(old + impulse, 0.0)
                impulse = acc - old
                c.normalImpulse[j] = acc
                if (impulse > c.maxNormalImpulse[j]) c.maxNormalImpulse[j] = impulse
                applyImpulse(a, b, rAx, rAy, rBx, rBy, impulse * nx, impulse * ny)
            }
        }
    }

    /**
     * After the solver: a grapple whose pull reached its break force, or a block that took its shed threshold to hold
     * squared up, gives way within the tick. Joint id order; these events come before the contact events.
     */
    private fun breakJoints() {
        val t = tick + 1
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            if (!j.checkBreak()) continue
            val grapple = j.type == JointType.Grapple
            val e = eventReader.add(if (grapple) EventReader.T_GRAPPLE_BROKEN else EventReader.T_BLOCK_SHED, t, j.a.id)
            eventReader.setInt(e, EventReader.I_B, j.b.id)
            eventReader.setInt(e, EventReader.I_JOINT, j.id)
            eventReader.setDouble(e, EventReader.D_VALUE, if (grapple) j.force() else j.lateralForce())
            removeJoint(j)
        }
    }

    // ---------------------------------------------------------------- ball: commands

    /** The ball, as of the end of the last step (or the last ball command). Always returns the same view. */
    fun ball(): BallView = ballView

    /** Wind (m/s) on the ground plane; drag acts on the ball's speed relative to it. */
    val windX: Double get() = flight.windX
    val windY: Double get() = flight.windY

    fun setWind(x: Double, y: Double) {
        require(x.isFinite() && y.isFinite()) { "wind must be finite: ($x, $y)" }
        flight.windX = x; flight.windY = y
        recorder?.onSetWind(tick, x, y)
    }

    /**
     * Puts the ball in the air at [from] (z ≥ 0 is its height) with [velocity]: a pass, a kick, a pitch or a fumble.
     * It takes the ball from its carrier (and from the ground, if it was loose). The values are copied.
     */
    fun launchBall(from: Vec3, velocity: Vec3, mode: FlightMode) {
        require(from.x.isFinite() && from.y.isFinite() && from.z.isFinite() && from.z >= 0.0) { "launch point must be finite with z >= 0: $from" }
        require(velocity.x.isFinite() && velocity.y.isFinite() && velocity.z.isFinite()) { "launch velocity must be finite: $velocity" }
        if (ball.mode == BallRecord.MODE_LOOSE) removeBallBody()
        carrier = null
        forwardProgressX = Double.NaN
        ball.flightMode = mode.ordinal
        takeOff(from.x, from.y, from.z, velocity.x, velocity.y, velocity.z)
        recorder?.onLaunchBall(tick, from, velocity, mode)
    }

    /**
     * A tip, bat or hit from game logic: adds [impulse] (N·s, on the ground plane) to a dynamic body's velocity and wakes
     * it. For the ball ([ballId]) it works in the air and on the ground, and [impulseZ] (≥ 0 on the ground) adds lift:
     * a loose ball batted upward goes back in the air. Other bodies take no impulseZ. Throws [IllegalStateException]
     * when the ball is Dead or Carried.
     */
    fun deflect(id: BodyId, impulse: Vec2, impulseZ: Double = 0.0) {
        val ix = impulse.x; val iy = impulse.y
        require(ix.isFinite() && iy.isFinite() && impulseZ.isFinite()) { "impulse must be finite: ($ix, $iy, $impulseZ)" }
        if (id.value == ballId.value) {
            val inv = 1.0 / config.ball.mass
            when (ball.mode) {
                BallRecord.MODE_AIRBORNE -> { ball.vx += ix * inv; ball.vy += iy * inv; ball.vz += impulseZ * inv }
                BallRecord.MODE_LOOSE -> {
                    val b = ballBody
                    if (impulseZ > 0.0) {
                        val vx = b.vx + ix * inv; val vy = b.vy + iy * inv
                        removeBallBody()
                        takeOff(ball.x, ball.y, 0.0, vx, vy, impulseZ * inv)
                    } else {
                        b.vx += ix * inv; b.vy += iy * inv
                        wake(b)
                        ball.vx = b.vx; ball.vy = b.vy
                    }
                }
                else -> throw IllegalStateException("No ball in play to deflect: the ball is ${ballView.state}")
            }
        } else {
            val b = live(id)
            require(b.type == BodyType.Dynamic) { "Only dynamic bodies can be deflected: body ${id.value} is ${b.type}" }
            require(impulseZ == 0.0) { "impulseZ is for the ball only: body ${id.value}" }
            b.vx += ix * b.invMass; b.vy += iy * b.invMass
            wake(b)
        }
        recorder?.onDeflect(tick, id, ix, iy, impulseZ)
    }

    /**
     * Gives a player a catch window: the ball in the air is catchable while it is within [reachRadius] (m) of his
     * center and between [BallConfig.catchMinHeight] and [reachHeight] (m; jumping raises it). Both 0 (the default)
     * means no window. The engine reports [EventType.CatchOpportunity]; the game decides the catch.
     */
    fun setReach(id: BodyId, reachRadius: Double, reachHeight: Double) {
        val b = gameBody(id)
        require(reachRadius.isFinite() && reachRadius >= 0.0) { "reachRadius must be finite and >= 0: $reachRadius" }
        require(reachHeight.isFinite() && reachHeight >= 0.0) { "reachHeight must be finite and >= 0: $reachHeight" }
        b.reachRadius = reachRadius; b.reachHeight = reachHeight
        recorder?.onSetReach(tick, id, reachRadius, reachHeight)
    }

    // ---------------------------------------------------------------- ball: launch helpers (read-only)

    // These fly a ball through the same drag model and ticks as step(), in the current wind, without goal posts, catch
    // windows or bounces. They change nothing and never allocate; "comes down through" a point means the ball descends
    // through the point's height there, so a target on the ground (z = 0) is where it lands.

    /**
     * The launch velocity from [from] whose arc peaks at height [apexHeight] (m above the ground, above both [from] and
     * [target]) and comes down through [target]. Written to [out], which is returned.
     */
    fun launchVelocityFor(from: Vec3, target: Vec3, apexHeight: Double, mode: FlightMode, out: Vec3): Vec3 {
        requirePoint(from); requirePoint(target)
        require(apexHeight.isFinite() && apexHeight > from.z && apexHeight > target.z) {
            "apexHeight must be above the launch (${from.z}) and the target (${target.z}): $apexHeight"
        }
        flight.solveApex(from.x, from.y, from.z, target.x, target.y, target.z, apexHeight, mode.ordinal)
        return out.set(flight.outX, flight.outY, flight.outZ)
    }

    /**
     * The launch velocity of magnitude [speed] from [from] that comes down through [target], on the lower arc or (with
     * [highArc]) the higher one. Writes [out] and returns true, or returns false (leaving [out] alone) when [target] is
     * out of reach at that speed.
     */
    fun launchVelocityFor(from: Vec3, target: Vec3, speed: Double, highArc: Boolean, mode: FlightMode, out: Vec3): Boolean {
        requirePoint(from); requirePoint(target)
        require(speed.isFinite() && speed > 0.0) { "speed must be finite and > 0: $speed" }
        if (!flight.solveSpeed(from.x, from.y, from.z, target.x, target.y, target.z, speed, highArc, mode.ordinal)) return false
        out.set(flight.outX, flight.outY, flight.outZ)
        return true
    }

    /**
     * Seconds after a launch from [from] at [velocity] until the ball has covered the ground distance to [target]
     * (along the line from [from] to it), for leading a receiver. NaN when it lands first.
     */
    fun timeToArrive(from: Vec3, velocity: Vec3, mode: FlightMode, target: Vec3): Double {
        requirePoint(from); requirePoint(target)
        val dx = target.x - from.x; val dy = target.y - from.y
        val d = DMath.hypot2(dx, dy)
        if (d == 0.0) return 0.0
        return flight.timeToDistance(from.x, from.y, from.z, velocity.x, velocity.y, velocity.z, mode.ordinal, dx / d, dy / d, d)
    }

    /**
     * Where a launch from [from] at [velocity] first reaches the ground, written to [out] (z = 0); returns the flight
     * time (s), or NaN (and NaN in [out]) when it stays up longer than a minute.
     */
    fun predictLanding(from: Vec3, velocity: Vec3, mode: FlightMode, out: Vec3): Double {
        requirePoint(from)
        if (!flight.predict(from.x, from.y, from.z, velocity.x, velocity.y, velocity.z, mode.ordinal, 0.0)) {
            out.set(Double.NaN, Double.NaN, Double.NaN)
            return Double.NaN
        }
        out.set(flight.hitX, flight.hitY, 0.0)
        return flight.hitTime
    }

    private fun requirePoint(p: Vec3) {
        require(p.x.isFinite() && p.y.isFinite() && p.z.isFinite() && p.z >= 0.0) { "point must be finite with z >= 0: $p" }
    }

    // ---------------------------------------------------------------- ball: simulation

    /** The ball leaves the ground or a hand: airborne from here, every catch window closed. */
    private fun takeOff(x: Double, y: Double, z: Double, vx: Double, vy: Double, vz: Double) {
        val s = ball
        s.mode = BallRecord.MODE_AIRBORNE
        s.x = x; s.y = y; s.z = z; s.vx = vx; s.vy = vy; s.vz = vz
        s.prevX = x; s.prevY = y; s.prevZ = z
        for (i in 0 until highWater) bodies[i]?.inCatch = false
    }

    private fun carryBall() {
        val c = carrier!!
        val s = ball
        s.x = c.x; s.y = c.y; s.z = config.ball.carryHeight
        s.vx = c.vx; s.vy = c.vy; s.vz = 0.0
    }

    /** A loose ball rolls to a stop: its ground speed drops by rollFriction·g each second. */
    private fun rollBall(b: Body) {
        val speed = DMath.hypot2(b.vx, b.vy)
        val dec = config.ball.rollFriction * config.gravity * WorldConfig.DT
        if (speed <= dec) { b.vx = 0.0; b.vy = 0.0 } else { val f = (speed - dec) / speed; b.vx *= f; b.vy *= f }
    }

    /** The ball lands for good: it becomes the loose-ball body at (x, y). Landing on a line or out of bounds is reported this tick. */
    private fun insertBallBody(x: Double, y: Double, vx: Double, vy: Double) {
        val b = ballBody
        b.x = x; b.y = y; b.prevX = x; b.prevY = y; b.x0 = x; b.y0 = y
        b.angle = 0.0; b.angle0 = 0.0
        b.vx = vx; b.vy = vy; b.angularVelocity = 0.0; b.preVx = vx; b.preVy = vy; b.fx = 0.0; b.fy = 0.0
        b.awake = true; b.sleepTime = 0.0
        b.lineMask = 0; b.outOfBounds = false
        b.geom.set(b.shape, x, y, 0.0)
        bodies[ballSlot] = b
        if (ballSlot >= highWater) highWater = ballSlot + 1
        val s = ball
        s.mode = BallRecord.MODE_LOOSE
        s.x = x; s.y = y; s.z = 0.0; s.vx = vx; s.vy = vy; s.vz = 0.0
    }

    /** The loose ball leaves the ground (picked up or put back in the air); bodies resting on it wake. */
    private fun removeBallBody() {
        val b = ballBody
        for (i in 0 until contactCount) {
            val c = contacts[i]
            if (c.a === b) wake(c.b!!) else if (c.b === b) wake(c.a!!)
        }
        val s = ball
        s.x = b.x; s.y = b.y; s.z = 0.0; s.vx = b.vx; s.vy = b.vy; s.vz = 0.0
        bodies[ballSlot] = null
    }

    /**
     * Phase 8. A ball in the air flies one tick (drag, wind, gravity), checking the goal posts, the end-line planes and
     * the ground along the way, then the catch windows. A loose ball mirrors its body; a carried one follows the carrier.
     * Events, in time order within the tick: KickResult (posts and planes) and BallBounce, then CatchOpportunity by id.
     */
    private fun ballFlight() {
        val s = ball
        s.prevX = s.x; s.prevY = s.y; s.prevZ = s.z
        when (s.mode) {
            BallRecord.MODE_AIRBORNE -> flyBall()
            BallRecord.MODE_LOOSE -> {
                val b = ballBody
                s.prevX = b.prevX; s.prevY = b.prevY
                s.x = b.x; s.y = b.y; s.z = 0.0; s.vx = b.vx; s.vy = b.vy; s.vz = 0.0
            }
            BallRecord.MODE_CARRIED -> carryBall()
        }
    }

    private fun flyBall() {
        val s = ball
        val p = fp
        p.set(s.x, s.y, s.z, s.vx, s.vy, s.vz)
        val k = flight.k(s.flightMode)
        val t1 = tick + 1
        var remaining = WorldConfig.DT
        var pass = 0
        while (remaining > 0.0 && pass < MAX_FLIGHT_PASSES) {
            pass++
            val x0 = p.x; val y0 = p.y; val z0 = p.z
            val vx0 = p.vx; val vy0 = p.vy; val vz0 = p.vz
            val tl = flight.tick(p, k, remaining, 0.0)
            val used = if (tl >= 0.0) tl else remaining
            // The goal posts, along the straight piece from the start to where the ball got (the ground, if it landed).
            val hit = if (config.goalPosts) posts.sweep(x0, y0, z0, p.x - x0, p.y - y0, p.z - z0) else -1
            if (hit >= 0) {
                val u = posts.t
                p.x = x0 + u * (p.x - x0); p.y = y0 + u * (p.y - y0); p.z = z0 + u * (p.z - z0)
                p.vx = vx0 + u * (p.vx - vx0); p.vy = vy0 + u * (p.vy - vy0); p.vz = vz0 + u * (p.vz - vz0)
                endLinePlanes(x0, y0, z0, p.x, p.y, p.z, t1)
                bounceOffPost(p, posts.nx, posts.ny, posts.nz)
                kickResult(KickResult.HitPost, p.x, p.y, p.z, if (hit < posts.count / 2) FieldLine.HOME_END_LINE else FieldLine.AWAY_END_LINE, t1)
                remaining -= u * used
                continue
            }
            endLinePlanes(x0, y0, z0, p.x, p.y, p.z, t1)
            remaining = if (tl >= 0.0) remaining - tl else 0.0
            if (tl >= 0.0 && bounceOnGround(p, t1)) return // it rolls on as the loose ball
        }
        if (p.z < 0.0) p.z = 0.0 // only after a post hit right at the ground used up the tick's pieces
        s.x = p.x; s.y = p.y; s.z = p.z; s.vx = p.vx; s.vy = p.vy; s.vz = p.vz
        catchWindows(t1)
    }

    /** Reflects the ball off a post (normal n, outward) with the ball–structure restitution and friction. */
    private fun bounceOffPost(p: FlightPoint, nx: Double, ny: Double, nz: Double) {
        val vn = p.vx * nx + p.vy * ny + p.vz * nz
        if (vn >= 0.0) return
        val mats = config.materials
        val e = mats.restitution(Material.BALL, Material.STRUCTURE)
        val mu = mats.friction(Material.BALL, Material.STRUCTURE)
        var tx = p.vx - vn * nx; var ty = p.vy - vn * ny; var tz = p.vz - vn * nz
        val vt = DMath.sqrt(tx * tx + ty * ty + tz * tz)
        if (vt > 0.0) {
            val f = (vt - DMath.min(mu * (1.0 + e) * -vn, ROLL_FRACTION * vt)) / vt
            tx *= f; ty *= f; tz *= f
        }
        p.vx = tx - e * vn * nx; p.vy = ty - e * vn * ny; p.vz = tz - e * vn * nz
    }

    /**
     * The ball reached the ground at [p]: it keeps e × its vertical speed (turf restitution × the surface's bounce),
     * loses ground speed to friction, and a real bounce (at least [BallConfig.minBounceSpeed] coming down) turns it by
     * a seeded random yaw, as a prolate ball does. Returns true when it stays down and rolls on as the loose ball.
     */
    private fun bounceOnGround(p: FlightPoint, t: Long): Boolean {
        val c = config.ball
        val mats = config.materials
        val vzIn = -p.vz // ≥ 0: coming down
        val e = mats.restitution(Material.BALL, Material.TURF) * surface.ballBounceMultiplier
        val mu = mats.friction(Material.BALL, Material.TURF)
        val ev = eventReader.add(EventReader.T_BALL_BOUNCE, t, ballId.value)
        setPointNormal(ev, p.x, p.y, 0.0, 0.0)
        eventReader.setDouble(ev, EventReader.D_VALUE, DMath.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz))
        val vh = DMath.hypot2(p.vx, p.vy)
        if (vh > 0.0) {
            val f = (vh - DMath.min(mu * (1.0 + e) * vzIn, ROLL_FRACTION * vh)) / vh
            p.vx *= f; p.vy *= f
        }
        if (vzIn >= c.minBounceSpeed && c.bounceYawMax > 0.0) {
            val yaw = rng.nextDouble(-c.bounceYawMax, c.bounceYawMax)
            val cs = DMath.cos(yaw); val sn = DMath.sin(yaw)
            val vx = p.vx
            p.vx = cs * vx - sn * p.vy
            p.vy = sn * vx + cs * p.vy
        }
        p.z = 0.0
        val up = e * vzIn
        if (up < c.minBounceSpeed) {
            insertBallBody(p.x, p.y, p.vx, p.vy)
            return true
        }
        p.vz = up
        return false
    }

    /** KickResult for each end-line plane the piece (x0, y0, z0)→(x1, y1, z1) crosses. */
    private fun endLinePlanes(x0: Double, y0: Double, z0: Double, x1: Double, y1: Double, z1: Double, t: Long) {
        plane(0.0, FieldLine.HOME_END_LINE, x0, y0, z0, x1, y1, z1, t)
        plane(field.lengthM, FieldLine.AWAY_END_LINE, x0, y0, z0, x1, y1, z1, t)
    }

    private fun plane(px: Double, line: Int, x0: Double, y0: Double, z0: Double, x1: Double, y1: Double, z1: Double, t: Long) {
        val dir = if (x0 < px && x1 >= px) 1 else if (x0 > px && x1 <= px) -1 else return
        val u = (px - x0) / (x1 - x0)
        val y = y0 + u * (y1 - y0)
        val z = z0 + u * (z1 - z0)
        val off = y - field.widthM / 2
        val half = field.uprightSpacingM / 2
        val result = when {
            off > half -> if (dir > 0) KickResult.WideLeft else KickResult.WideRight
            off < -half -> if (dir > 0) KickResult.WideRight else KickResult.WideLeft
            z > field.crossbarHeightM -> KickResult.Good
            else -> KickResult.Short
        }
        kickResult(result, px, y, z, line, t)
    }

    private fun kickResult(result: KickResult, x: Double, y: Double, z: Double, line: Int, t: Long) {
        val e = eventReader.add(EventReader.T_KICK_RESULT, t, ballId.value)
        eventReader.setInt(e, EventReader.I_KICK, result.ordinal)
        eventReader.setInt(e, EventReader.I_LINE, line)
        setPointNormal(e, x, y, 0.0, 0.0)
        eventReader.setDouble(e, EventReader.D_PZ, z)
    }

    /** CatchOpportunity for each player the ball has just come within reach of, in id order. */
    private fun catchWindows(t: Long) {
        val s = ball
        val minZ = config.ball.catchMinHeight
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.reachRadius <= 0.0) continue
            val dx = s.x - b.x; val dy = s.y - b.y
            val inside = s.z >= minZ && s.z <= b.reachHeight && dx * dx + dy * dy <= b.reachRadius * b.reachRadius
            if (inside && !b.inCatch) {
                val e = eventReader.add(EventReader.T_CATCH, t, b.id)
                setPointNormal(e, s.x, s.y, 0.0, 0.0)
                eventReader.setDouble(e, EventReader.D_PZ, s.z)
                val vh = DMath.hypot2(s.vx, s.vy)
                eventReader.setDouble(e, EventReader.D_VALUE, DMath.sqrt(vh * vh + s.vz * s.vz))
                eventReader.setDouble(e, EventReader.D_ANGLE_A, DMath.wrapPi(DMath.atan2(s.vy, s.vx) - b.angle))
                eventReader.setDouble(e, EventReader.D_ANGLE_B, DMath.atan2(-s.vz, vh))
            }
            b.inCatch = inside
        }
    }

    // ---------------------------------------------------------------- continuous collision (2D)

    private val sweepHit = RayResult()

    /**
     * After the solver: the loose ball and every `bullet` body (circles) sweep from where they started the step to where
     * they ended against static segments and polygons (static bodies, runoff walls, post bases). A body that would have
     * passed through one stops where it first touches, and bounces with the pair's restitution. Bodies that start the
     * step touching are left to the contact solver.
     */
    private fun continuousCollision() {
        val threshold = config.restitutionThreshold
        val mats = config.materials
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (!b.bullet || b.type != BodyType.Dynamic || !b.awake) continue
            val shape = b.shape
            if (shape !is ShapeDef.Circle || shape.sensor) continue
            val dx = b.x - b.x0; val dy = b.y - b.y0
            if (dx == 0.0 && dy == 0.0) continue
            val r = DMath.max(shape.radius - TOUCH_SLOP, 0.5 * shape.radius)
            val minX = DMath.min(b.x0, b.x) - r; val maxX = DMath.max(b.x0, b.x) + r
            val minY = DMath.min(b.y0, b.y) - r; val maxY = DMath.max(b.y0, b.y) + r
            var best = 2.0
            var nx = 0.0; var ny = 0.0; var mat = 0
            for (p in 0 until highWater + walls.size) {
                val o = if (p < highWater) bodies[p] else if (wallOn[p - highWater]) walls[p - highWater] else null
                if (o == null || o.type != BodyType.Static || o.sensor) continue
                if ((b.categoryBits and o.maskBits) == 0 || (o.categoryBits and b.maskBits) == 0) continue
                val g = o.geom
                if (g.minX > maxX || g.maxX < minX || g.minY > maxY || g.maxY < minY) continue
                if (!RayCast.cast(g, r, b.x0, b.y0, dx, dy, sweepHit)) continue
                if (sweepHit.t <= 0.0 || sweepHit.t >= best) continue
                best = sweepHit.t; nx = sweepHit.nx; ny = sweepHit.ny; mat = o.material
            }
            if (best > 1.0) continue
            b.x = b.x0 + best * dx; b.y = b.y0 + best * dy
            val vn = b.vx * nx + b.vy * ny
            if (vn < 0.0) {
                val e = if (-vn > threshold) mats.restitution(b.material, mat) else 0.0
                b.vx -= (1.0 + e) * vn * nx; b.vy -= (1.0 + e) * vn * ny
            }
        }
    }

    // ---------------------------------------------------------------- events phase

    // Pairs that were touching at the end of the last step, sorted by contact key. A pair stays here while both
    // bodies sleep (sleeping pairs are not re-collided), so falling asleep is not a ContactEnd. Double-buffered.
    private val touchCap = 2 * config.maxContacts
    private var touchKey = LongArray(touchCap)
    private var touchA = IntArray(touchCap)
    private var touchB = IntArray(touchCap)
    private var touchSensor = BooleanArray(touchCap)
    private var touchCount = 0
    private var nextKey = LongArray(touchCap)
    private var nextA = IntArray(touchCap)
    private var nextB = IntArray(touchCap)
    private var nextSensor = BooleanArray(touchCap)
    private var nextCount = 0

    /**
     * Phase 9. Refreshes world-space shapes at the new positions (queries and the next broad phase use them), then
     * reports contact begin/end (with Hit and Trip), field-line sensors, out of bounds and goal-line crossings.
     * Events are written in a fixed order: contacts by key, then each body's line events by ascending id.
     */
    private fun emitEvents() {
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Static) b.geom.set(b.shape, b.x, b.y, b.angle)
        }
        val c = carrier
        if (c != null && c.stance == Stance.Upright && attackDirection * (c.x - forwardProgressX) > 0.0) forwardProgressX = c.x
        emitContactEvents()
        emitLineEvents()
    }

    private fun emitContactEvents() {
        val t = tick + 1
        nextCount = 0
        var i = 0
        var j = 0
        while (i < contactCount || j < touchCount) {
            val keyC = if (i < contactCount) contacts[i].key else Long.MAX_VALUE
            val keyO = if (j < touchCount) touchKey[j] else Long.MAX_VALUE
            if (keyC < keyO) {
                val c = contacts[i++]
                if (touching(c)) { keepTouching(c.key, c.idA, c.idB, c.sensor); contactBegin(c, t) }
            } else if (keyO < keyC) {
                // Touching last tick, not even a candidate now: either both sides sleep (still touching) or it ended.
                if (asleepPair(touchA[j], touchB[j])) keepTouching(touchKey[j], touchA[j], touchB[j], touchSensor[j])
                else contactEnd(touchA[j], touchB[j], touchSensor[j], t)
                j++
            } else {
                val c = contacts[i++]
                if (touching(c)) keepTouching(c.key, c.idA, c.idB, c.sensor)
                else contactEnd(touchA[j], touchB[j], touchSensor[j], t)
                j++
            }
        }
        val k = touchKey; touchKey = nextKey; nextKey = k
        val a = touchA; touchA = nextA; nextA = a
        val b = touchB; touchB = nextB; nextB = b
        val s = touchSensor; touchSensor = nextSensor; nextSensor = s
        touchCount = nextCount
    }

    private fun keepTouching(key: Long, a: Int, b: Int, sensor: Boolean) {
        if (nextCount == touchCap) return // over capacity: the pair is forgotten and may begin again later
        nextKey[nextCount] = key; nextA[nextCount] = a; nextB[nextCount] = b; nextSensor[nextCount] = sensor
        nextCount++
    }

    /** The body with id [id] if it is alive, else null. Walls (negative ids) are not bodies: null. */
    private fun liveOrNull(id: Int): Body? {
        if (id < 0) return null
        val slot = BodyId(id).slot
        if (slot >= highWater) return null
        val b = bodies[slot]
        return if (b != null && b.id == id) b else null
    }

    private fun asleepPair(a: Int, b: Int): Boolean {
        val ba = liveOrNull(a) ?: return false
        if (ba.active) return false
        if (b < 0) return true // a runoff wall never moves
        val bb = liveOrNull(b) ?: return false
        return !bb.active
    }

    /**
     * Touching after this step: the solver pushed (normal impulse), or a point's separation, moved along with the
     * bodies since the narrow phase, is below [TOUCH_SLOP]. Speculative contacts that never closed are not touching.
     */
    private fun touching(c: Contact): Boolean {
        val a = c.a!!; val b = c.b!!
        val m = c.m
        val dpx = (b.x - b.x0) - (a.x - a.x0)
        val dpy = (b.y - b.y0) - (a.y - a.y0)
        val daA = a.angle - a.angle0
        val daB = b.angle - b.angle0
        for (j in 0 until m.count) {
            if (c.solve && c.normalImpulse[j] > 0.0) return true
            val dx = dpx - daB * (m.py[j] - b.y0) + daA * (m.py[j] - a.y0)
            val dy = dpy + daB * (m.px[j] - b.x0) - daA * (m.px[j] - a.x0)
            if (m.sep[j] + dx * m.nx + dy * m.ny < TOUCH_SLOP) return true
        }
        return false
    }

    private fun contactBegin(c: Contact, t: Long) {
        val ev = eventReader
        val m = c.m
        var px = 0.0; var py = 0.0
        for (j in 0 until m.count) { px += m.px[j]; py += m.py[j] }
        px /= m.count; py /= m.count
        val e = ev.add(EventReader.T_CONTACT_BEGIN, t, c.idA)
        ev.setInt(e, EventReader.I_B, c.idB)
        ev.setInt(e, EventReader.I_WALL, c.wall)
        ev.setInt(e, EventReader.I_SENSOR, if (c.sensor) 1 else 0)
        setPointNormal(e, px, py, m.nx, m.ny)
        if (c.sensor || c.wall != ContactView.NO_WALL) return
        val a = c.a!!; val b = c.b!!
        if (isPlayer(a) && isPlayer(b)) hit(c, a, b, px, py, t)
        if (b.stance == Stance.Down && a.stance != Stance.Down && isPlayer(a)) trip(a, b, px, py, m.nx, m.ny, t)
        else if (a.stance == Stance.Down && b.stance != Stance.Down && isPlayer(b)) trip(b, a, px, py, -m.nx, -m.ny, t)
    }

    private fun isPlayer(b: Body): Boolean = b.type == BodyType.Dynamic && (b.categoryBits and Category.PLAYERS) != 0

    private fun setPointNormal(e: Int, px: Double, py: Double, nx: Double, ny: Double) {
        val ev = eventReader
        ev.setDouble(e, EventReader.D_PX, px); ev.setDouble(e, EventReader.D_PY, py)
        ev.setDouble(e, EventReader.D_NX, nx); ev.setDouble(e, EventReader.D_NY, ny)
    }

    private fun hit(c: Contact, a: Body, b: Body, px: Double, py: Double, t: Long) {
        val ev = eventReader
        val nx = c.m.nx; val ny = c.m.ny
        var impulse = 0.0
        for (j in 0 until c.m.count) impulse += c.normalImpulse[j]
        val closing = -((b.preVx - a.preVx) * nx + (b.preVy - a.preVy) * ny)
        val e = ev.add(EventReader.T_HIT, t, a.id)
        ev.setInt(e, EventReader.I_B, b.id)
        setPointNormal(e, px, py, nx, ny)
        ev.setDouble(e, EventReader.D_CLOSING, DMath.max(closing, 0.0))
        ev.setDouble(e, EventReader.D_IMPULSE, impulse)
        // A meets the contact along +n, B along −n. Headings are from the start of the step, before impact.
        ev.setDouble(e, EventReader.D_ANGLE_A, DMath.wrapPi(DMath.atan2(ny, nx) - a.angle0))
        ev.setDouble(e, EventReader.D_ANGLE_B, DMath.wrapPi(DMath.atan2(-ny, -nx) - b.angle0))
        ev.setDouble(e, EventReader.D_MAX, a.mass * a.preVx); ev.setDouble(e, EventReader.D_MAY, a.mass * a.preVy)
        ev.setDouble(e, EventReader.D_MBX, b.mass * b.preVx); ev.setDouble(e, EventReader.D_MBY, b.mass * b.preVy)
    }

    private fun trip(mover: Body, down: Body, px: Double, py: Double, nx: Double, ny: Double, t: Long) {
        val speed = DMath.hypot2(mover.preVx, mover.preVy)
        if (speed < config.tripMinSpeed) return
        val ev = eventReader
        val e = ev.add(EventReader.T_TRIP, t, mover.id)
        ev.setInt(e, EventReader.I_B, down.id)
        setPointNormal(e, px, py, nx, ny)
        ev.setDouble(e, EventReader.D_VALUE, speed)
    }

    private fun contactEnd(a: Int, b: Int, sensor: Boolean, t: Long) {
        val ev = eventReader
        val e = ev.add(EventReader.T_CONTACT_END, t, a)
        ev.setInt(e, EventReader.I_B, b)
        ev.setInt(e, EventReader.I_WALL, if (b < 0) -1 - b else ContactView.NO_WALL)
        ev.setInt(e, EventReader.I_SENSOR, if (sensor) 1 else 0)
    }

    // Field lines are axis-aligned, so a convex shape crosses one exactly when its bounds straddle it. The only
    // approximation is at the four field corners, where a rotated capsule's or polygon's bounds may reach past
    // the end of a line its shape does not quite touch.

    private fun lineOverlap(g: Geom): Int {
        val len = field.lengthM; val wid = field.widthM
        val alongX = g.maxX >= 0.0 && g.minX <= len
        val alongY = g.maxY >= 0.0 && g.minY <= wid
        var m = 0
        if (alongX && g.minY <= 0.0 && g.maxY >= 0.0) m = m or (1 shl FieldLine.NEAR_SIDELINE)
        if (alongX && g.minY <= wid && g.maxY >= wid) m = m or (1 shl FieldLine.FAR_SIDELINE)
        if (alongY && g.minX <= 0.0 && g.maxX >= 0.0) m = m or (1 shl FieldLine.HOME_END_LINE)
        if (alongY && g.minX <= len && g.maxX >= len) m = m or (1 shl FieldLine.AWAY_END_LINE)
        val hg = field.homeGoalLineX; val ag = field.awayGoalLineX
        if (alongY && g.minX <= hg && g.maxX >= hg) m = m or (1 shl FieldLine.HOME_GOAL_LINE)
        if (alongY && g.minX <= ag && g.maxX >= ag) m = m or (1 shl FieldLine.AWAY_GOAL_LINE)
        return m
    }

    /** Touching a sideline or end line is out. */
    private fun outOfBounds(g: Geom): Boolean =
        g.minY <= 0.0 || g.maxY >= field.widthM || g.minX <= 0.0 || g.maxX >= field.lengthM

    private fun emitLineEvents() {
        val t = tick + 1
        val ev = eventReader
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type == BodyType.Static || (b.maskBits and Category.LINE_SENSOR) == 0) continue
            val g = b.geom
            val mask = lineOverlap(g)
            val changed = mask xor b.lineMask
            if (changed != 0) {
                for (line in 0 until FieldLine.COUNT) {
                    if ((changed and (1 shl line)) == 0) continue
                    val e = ev.add(if ((mask and (1 shl line)) != 0) EventReader.T_SENSOR_BEGIN else EventReader.T_SENSOR_END, t, b.id)
                    ev.setInt(e, EventReader.I_LINE, line)
                }
                b.lineMask = mask
            }
            val out = outOfBounds(g)
            if (out && !b.outOfBounds) {
                val line = when {
                    g.minY <= 0.0 -> FieldLine.NEAR_SIDELINE
                    g.maxY >= field.widthM -> FieldLine.FAR_SIDELINE
                    g.minX <= 0.0 -> FieldLine.HOME_END_LINE
                    else -> FieldLine.AWAY_END_LINE
                }
                val e = ev.add(EventReader.T_OUT_OF_BOUNDS, t, b.id)
                ev.setInt(e, EventReader.I_LINE, line)
            }
            b.outOfBounds = out
            goalLine(b, field.homeGoalLineX, FieldLine.HOME_GOAL_LINE, t)
            goalLine(b, field.awayGoalLineX, FieldLine.AWAY_GOAL_LINE, t)
        }
    }

    /** The center crossed x = [gx] this step, between the sidelines. The line itself counts as the far side. */
    private fun goalLine(b: Body, gx: Double, line: Int, t: Long) {
        val x0 = b.prevX; val x1 = b.x
        val dir = if (x0 < gx && x1 >= gx) 1 else if (x0 >= gx && x1 < gx) -1 else return
        val y = b.prevY + (gx - x0) / (x1 - x0) * (b.y - b.prevY)
        if (y < 0.0 || y > field.widthM) return
        val e = eventReader.add(EventReader.T_GOAL_LINE, t, b.id)
        eventReader.setInt(e, EventReader.I_LINE, line)
        eventReader.setInt(e, EventReader.I_DIR, dir)
        setPointNormal(e, gx, y, 0.0, 0.0)
    }

    private fun bookkeeping() {
        updateSleep()
        tick++
        if (config.hashEveryTick) lastTickHash = stateHash()
    }

    // ---------------------------------------------------------------- sleeping

    private fun islandRoot(i: Int): Int {
        var r = i
        while (islandParent[r] != r) r = islandParent[r]
        var j = i
        while (islandParent[j] != r) { val next = islandParent[j]; islandParent[j] = r; j = next }
        return r
    }

    /**
     * A body is sleepy after [WorldConfig.sleepTimeS] below [WorldConfig.sleepSpeed]. Touching awake bodies form
     * an island, and an island sleeps only when all of its bodies are sleepy, so a resting pile never wakes
     * itself up one body at a time. A dynamic body touching a moving kinematic body never gets sleepy.
     */
    private fun updateSleep() {
        val dt = WorldConfig.DT
        val vMax = config.sleepSpeed
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic || !b.awake) continue
            val fast = b.vx * b.vx + b.vy * b.vy > vMax * vMax || DMath.abs(b.angularVelocity) * b.extent > vMax ||
                b.loco?.wantsMotion == true // a player trying to move never falls asleep
            b.sleepTime = if (fast) 0.0 else b.sleepTime + dt
            islandParent[i] = i
            islandMinTime[i] = Double.MAX_VALUE
        }
        for (k in 0 until contactCount) {
            val c = contacts[k]
            if (c.sensor) continue
            val a = c.a!!; val b = c.b!!
            val aLive = a.type == BodyType.Dynamic && a.awake
            val bLive = b.type == BodyType.Dynamic && b.awake
            if (aLive && bLive) {
                val ra = islandRoot(BodyId(a.id).slot)
                val rb = islandRoot(BodyId(b.id).slot)
                if (ra < rb) islandParent[rb] = ra else if (rb < ra) islandParent[ra] = rb
            } else if (aLive && b.type == BodyType.Kinematic && b.active) {
                a.sleepTime = 0.0
            } else if (bLive && a.type == BodyType.Kinematic && a.active) {
                b.sleepTime = 0.0
            }
        }
        // Jointed bodies share an island too; an engaged block never sleeps.
        for (k in 0 until jointHighWater) {
            val j = joints[k] ?: continue
            val a = j.a; val b = j.b
            if (j.type == JointType.Block) { a.sleepTime = 0.0; b.sleepTime = 0.0 }
            val aLive = a.type == BodyType.Dynamic && a.awake
            val bLive = b.type == BodyType.Dynamic && b.awake
            if (aLive && bLive) {
                val ra = islandRoot(BodyId(a.id).slot)
                val rb = islandRoot(BodyId(b.id).slot)
                if (ra < rb) islandParent[rb] = ra else if (rb < ra) islandParent[ra] = rb
            } else if (aLive && b.type == BodyType.Kinematic && b.active) {
                a.sleepTime = 0.0
            } else if (bLive && a.type == BodyType.Kinematic && a.active) {
                b.sleepTime = 0.0
            }
        }
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic || !b.awake) continue
            val r = islandRoot(i)
            if (b.sleepTime < islandMinTime[r]) islandMinTime[r] = b.sleepTime
        }
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic || !b.awake) continue
            if (islandMinTime[islandRoot(i)] >= config.sleepTimeS) {
                b.awake = false
                b.vx = 0.0; b.vy = 0.0; b.angularVelocity = 0.0
            }
        }
    }

    // ---------------------------------------------------------------- determinism

    /** The hash computed at the end of the last tick, when [WorldConfig.hashEveryTick] is on. */
    var lastTickHash: Long = 0L
        private set

    /**
     * FNV-1a over the raw bits of every live body's id, position, angle and velocities (ascending id),
     * then the ball state, then the tick count. Equal hashes on two devices mean equal simulations.
     */
    fun stateHash(): Long {
        var h = Fnv1a64.OFFSET
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            h = Fnv1a64.mixLong(h, b.id.toLong())
            h = Fnv1a64.mixDouble(h, b.x)
            h = Fnv1a64.mixDouble(h, b.y)
            h = Fnv1a64.mixDouble(h, b.angle)
            h = Fnv1a64.mixDouble(h, b.vx)
            h = Fnv1a64.mixDouble(h, b.vy)
            h = Fnv1a64.mixDouble(h, b.angularVelocity)
        }
        h = Fnv1a64.mixDouble(h, ball.x)
        h = Fnv1a64.mixDouble(h, ball.y)
        h = Fnv1a64.mixDouble(h, ball.z)
        h = Fnv1a64.mixDouble(h, ball.vx)
        h = Fnv1a64.mixDouble(h, ball.vy)
        h = Fnv1a64.mixDouble(h, ball.vz)
        h = Fnv1a64.mixLong(h, ball.mode.toLong())
        h = Fnv1a64.mixLong(h, ball.flightMode.toLong())
        h = Fnv1a64.mixLong(h, tick)
        return h
    }

    /** The engine's own statics, indexed by [ContactView] wall number: four runoff walls, then the two post bases. */
    private fun makeWalls(): Array<Body> {
        val f = field
        val x0 = -f.runoffM
        val y0 = -f.runoffM
        val x1 = f.lengthM + f.runoffM
        val y1 = f.widthM + f.runoffM
        val base = ShapeDef.box(f.goalPostBaseHalfM, f.goalPostBaseHalfM)
        val shapes = arrayOf(
            ShapeDef.Segment(x0, y0, x1, y0), // WALL_NEAR
            ShapeDef.Segment(x0, y1, x1, y1), // WALL_FAR
            ShapeDef.Segment(x0, y0, x0, y1), // WALL_HOME
            ShapeDef.Segment(x1, y0, x1, y1), // WALL_AWAY
            base, // POST_HOME
            base, // POST_AWAY
        )
        return Array(shapes.size) { k ->
            val w = Body(-1 - k, BodyType.Static, shapes[k])
            w.categoryBits = Category.STATIC
            w.maskBits = Category.ALL
            w.material = Material.STRUCTURE
            w.extent = shapeExtent(w.shape)
            if (k == ContactView.POST_HOME) { w.x = -f.goalPostOffsetM; w.y = f.widthM / 2 }
            if (k == ContactView.POST_AWAY) { w.x = f.lengthM + f.goalPostOffsetM; w.y = f.widthM / 2 }
            w.geom.set(w.shape, w.x, w.y, 0.0)
            w
        }
    }

    private companion object {
        /** 1 ns of slack so frame times that sum to whole ticks are not lost to rounding. */
        const val ADVANCE_EPS = 1e-9

        /** Lying capsule for Diving and Down, in multiples of the upright radius: 0.34 m → 1.77 m long, 0.20 m thick. */
        const val LYING_HALF_LENGTH = 2.0
        const val LYING_RADIUS = 0.6

        /** Separation (m) below which a contact counts as touching for events (Box2D's linear slop). */
        const val TOUCH_SLOP = 0.005

        /** Runoff walls sort after every body id (ids are non-negative Ints). */
        const val WALL_KEY_BASE = 0x80000000L

        /** Spin damping of the loose ball (it has no use for spin; this keeps wall glances from leaving it twirling). */
        const val BALL_SPIN_DAMPING = 2.0

        /**
         * A bounce's friction can take at most this share of the slip speed along the surface: the point where a hollow
         * shell (I = ⅔mr²) starts to roll instead of slide, 1 / (1 + mr²/I) = 0.4.
         */
        const val ROLL_FRACTION = 0.4

        /** Straight pieces the ball's tick may be split into by post hits and bounces. */
        const val MAX_FLIGHT_PASSES = 4

        /** Largest distance from the body origin to any point of [shape]. */
        fun shapeExtent(shape: ShapeDef): Double = when (shape) {
            is ShapeDef.Circle -> shape.radius
            is ShapeDef.Capsule -> shape.halfLength + shape.radius
            is ShapeDef.Segment -> DMath.max(DMath.hypot2(shape.x1, shape.y1), DMath.hypot2(shape.x2, shape.y2))
            is ShapeDef.Polygon -> {
                var r = 0.0
                for (i in 0 until shape.count) r = DMath.max(r, DMath.hypot2(shape.vertex(2 * i), shape.vertex(2 * i + 1)))
                r
            }
        }

        /** Rotational inertia about the body origin (the center of mass) for uniform density. */
        fun inertiaAboutOrigin(shape: ShapeDef, mass: Double): Double = when (shape) {
            is ShapeDef.Circle -> 0.5 * mass * shape.radius * shape.radius
            is ShapeDef.Capsule -> {
                val h = shape.halfLength
                val r = shape.radius
                val rectArea = 4.0 * h * r
                val circleArea = DMath.PI * r * r
                val mRect = mass * rectArea / (rectArea + circleArea)
                val mCircle = mass - mRect
                // Each half-disk's centroid sits 4r/(3π) beyond its segment end.
                val lc = 4.0 * r / (3.0 * DMath.PI)
                mRect * (4.0 * h * h + 4.0 * r * r) / 12.0 + mCircle * (0.5 * r * r + h * h + 2.0 * h * lc)
            }
            is ShapeDef.Polygon -> {
                var area2 = 0.0
                var sum = 0.0
                val n = shape.count
                for (i in 0 until n) {
                    val j = if (i + 1 == n) 0 else i + 1
                    val x1 = shape.vertex(2 * i); val y1 = shape.vertex(2 * i + 1)
                    val x2 = shape.vertex(2 * j); val y2 = shape.vertex(2 * j + 1)
                    val cross = x1 * y2 - x2 * y1
                    area2 += cross
                    sum += cross * (x1 * x1 + x1 * x2 + x2 * x2 + y1 * y1 + y1 * y2 + y2 * y2)
                }
                // I = (ρ/12) Σ cross·(…) with ρ = mass / area = 2·mass / area2.
                mass * sum / (6.0 * area2)
            }
            is ShapeDef.Segment -> 0.0 // static only
        }
    }
}
