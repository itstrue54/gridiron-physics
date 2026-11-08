package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BallState
import io.github.itstrue54.gridironphysics.internal.Body
import io.github.itstrue54.gridironphysics.internal.BroadPhase
import io.github.itstrue54.gridironphysics.internal.BruteForceBroadPhase
import io.github.itstrue54.gridironphysics.internal.Collide
import io.github.itstrue54.gridironphysics.internal.Contact
import io.github.itstrue54.gridironphysics.internal.Geom
import io.github.itstrue54.gridironphysics.internal.LocomotionController
import io.github.itstrue54.gridironphysics.internal.RayCast
import io.github.itstrue54.gridironphysics.internal.RayResult
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Fnv1a64
import io.github.itstrue54.gridironphysics.math.SplitMix64
import io.github.itstrue54.gridironphysics.math.Vec2

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
    internal val ball = BallState()

    init {
        require(config.maxBodies <= BodyId.MAX_SLOTS) { "maxBodies must be <= ${BodyId.MAX_SLOTS}" }
    }

    /** Slot-indexed storage. Iterating slots in order is iterating bodies in ascending id order. */
    private val bodies = arrayOfNulls<Body>(config.maxBodies)
    private val generations = IntArray(config.maxBodies)
    private var liveCount = 0

    /** One past the highest slot ever used; loops stop here. */
    private var highWater = 0
    private val view = BodyView()

    // ---------------------------------------------------------------- collision state (all preallocated)

    /** Runoff walls: engine-owned static geometry, not bodies (they never appear in [bodyIds]). */
    private val walls: Array<Body> = makeWalls()
    private val wallCount = if (config.runoffWalls) walls.size else 0
    private val broad: BroadPhase = BruteForceBroadPhase()
    private val proxyCap = config.maxBodies + walls.size
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
    private val islandParent = IntArray(config.maxBodies)
    private val islandMinTime = DoubleArray(config.maxBodies)

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
        val dead = live(id)
        // Bodies resting against this one must not stay asleep against nothing.
        for (i in 0 until contactCount) {
            val c = contacts[i]
            if (c.a === dead) wake(c.b!!) else if (c.b === dead) wake(c.a!!)
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

    /** Number of live bodies. */
    val bodyCount: Int get() = liveCount

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
        require(id.value >= 0 && slot < highWater) { "Unknown body id ${id.value}" }
        val b = bodies[slot]
        require(b != null && b.id == id.value) { "Body ${id.value} was destroyed" }
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
        val b = live(id)
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
        val b = live(id)
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
        val b = live(id)
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
    private val nearDist = DoubleArray(config.maxBodies)

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
        applyIntents()
        integrateVelocities()
        broadPhase()
        narrowPhase()
        solveVelocities()
        integratePositions()
        relax()
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

    private fun integrateVelocities() {
        val dt = WorldConfig.DT
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic || !b.awake) continue
            b.vx += b.fx * b.invMass * dt
            b.vy += b.fy * b.invMass * dt
            b.fx = 0.0; b.fy = 0.0
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
        val n = highWater + wallCount
        for (p in 0 until n) {
            val b = if (p < highWater) bodies[p] else walls[p - highWater]
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

    /** Prepares every contact, applies the warm-start impulses, then runs the velocity iterations with soft bias. */
    private fun solveVelocities() {
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
        for (it in 0 until config.relaxIterations) {
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

    private fun ballFlight() {
        // M6: 2.5D ball.
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
        h = Fnv1a64.mixLong(h, tick)
        return h
    }

    private fun makeWalls(): Array<Body> {
        val f = field
        val x0 = -f.runoffM
        val y0 = -f.runoffM
        val x1 = f.lengthM + f.runoffM
        val y1 = f.widthM + f.runoffM
        val shapes = arrayOf(
            ShapeDef.Segment(x0, y0, x1, y0), // WALL_NEAR
            ShapeDef.Segment(x0, y1, x1, y1), // WALL_FAR
            ShapeDef.Segment(x0, y0, x0, y1), // WALL_HOME
            ShapeDef.Segment(x1, y0, x1, y1), // WALL_AWAY
        )
        return Array(shapes.size) { k ->
            val w = Body(-1 - k, BodyType.Static, shapes[k])
            w.categoryBits = Category.STATIC
            w.maskBits = Category.ALL
            w.material = Material.STRUCTURE
            w.extent = shapeExtent(w.shape)
            w.geom.set(w.shape, 0.0, 0.0, 0.0)
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
