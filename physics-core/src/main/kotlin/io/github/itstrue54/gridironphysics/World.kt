package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BallState
import io.github.itstrue54.gridironphysics.internal.Body
import io.github.itstrue54.gridironphysics.math.Fnv1a64
import io.github.itstrue54.gridironphysics.math.SplitMix64

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
        val r = b.radius
        if (def.type == BodyType.Dynamic) {
            b.mass = def.mass
            b.invMass = 1.0 / def.mass
            b.inertia = 0.5 * def.mass * r * r
            b.invInertia = 1.0 / b.inertia
        }
        bodies[slot] = b
        liveCount++
        if (slot >= highWater) highWater = slot + 1
        recorder?.onCreateBody(tick, id, def)
        return id
    }

    fun destroyBody(id: BodyId) {
        live(id)
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

    private fun applyIntents() {
        // M3: locomotion controllers turn intents into forces.
    }

    private fun integrateVelocities() {
        val dt = WorldConfig.DT
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            if (b.type != BodyType.Dynamic) continue
            // Forces arrive in M3. Damping uses the Box2D form, stable for any damping value.
            b.vx *= 1.0 / (1.0 + dt * b.linearDamping)
            b.vy *= 1.0 / (1.0 + dt * b.linearDamping)
            b.angularVelocity *= 1.0 / (1.0 + dt * b.angularDamping)
        }
    }

    private fun broadPhase() {
        // M2: candidate pairs sorted by (minId, maxId).
    }

    private fun narrowPhase() {
        // M2: contact manifolds.
    }

    private fun solveVelocities() {
        // M2: warm start + contact/joint iterations.
    }

    private fun integratePositions() {
        val dt = WorldConfig.DT
        for (i in 0 until highWater) {
            val b = bodies[i] ?: continue
            b.prevX = b.x
            b.prevY = b.y
            if (b.type == BodyType.Static) continue
            b.x += b.vx * dt
            b.y += b.vy * dt
            b.angle += b.angularVelocity * dt
        }
    }

    private fun relax() {
        // M2: soft position correction.
    }

    private fun ballFlight() {
        // M6: 2.5D ball.
    }

    private fun emitEvents() {
        // M4: event ring buffer.
    }

    private fun bookkeeping() {
        // M2: sleep states.
        tick++
        if (config.hashEveryTick) lastTickHash = stateHash()
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

    private companion object {
        /** 1 ns of slack so frame times that sum to whole ticks are not lost to rounding. */
        const val ADVANCE_EPS = 1e-9
    }
}
