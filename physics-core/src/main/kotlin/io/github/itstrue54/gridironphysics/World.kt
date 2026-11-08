package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BallState
import io.github.itstrue54.gridironphysics.internal.Body
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

    private val bodies = arrayOfNulls<Body>(config.maxBodies)
    private var nextId = 0
    private val view = BodyView()

    // ---------------------------------------------------------------- bodies

    fun createBody(def: BodyDef): BodyId {
        def.validate()
        check(nextId < config.maxBodies) { "World is full: ${config.maxBodies} body ids issued (ids are never reused)" }
        val b = Body(nextId, def.type, def.shape)
        b.x = def.x; b.y = def.y; b.prevX = def.x; b.prevY = def.y
        b.angle = def.angle
        b.vx = def.vx; b.vy = def.vy; b.angularVelocity = def.angularVelocity
        b.linearDamping = def.linearDamping; b.angularDamping = def.angularDamping
        b.userData = def.userData; b.bullet = def.bullet
        val r = b.radius
        if (def.type == BodyType.Dynamic) {
            b.mass = def.mass
            b.invMass = 1.0 / def.mass
            b.inertia = 0.5 * def.mass * r * r
            b.invInertia = 1.0 / b.inertia
        }
        bodies[nextId] = b
        val id = BodyId(nextId++)
        recorder?.onCreateBody(tick, id, def)
        return id
    }

    fun destroyBody(id: BodyId) {
        live(id)
        bodies[id.value] = null
        recorder?.onDestroyBody(tick, id)
    }

    /** Read-only view of a body. The returned object is reused by the next call. */
    fun body(id: BodyId): BodyView {
        view.b = live(id)
        return view
    }

    /** Writes live body ids in ascending order into [out]; returns how many. */
    fun bodyIds(out: IntArray): Int {
        var n = 0
        for (i in 0 until nextId) {
            if (bodies[i] != null) {
                if (n < out.size) out[n] = i
                n++
            }
        }
        return minOf(n, out.size)
    }

    private fun live(id: BodyId): Body {
        val i = id.value
        require(i in 0 until nextId) { "Unknown body id $i" }
        return bodies[i] ?: throw IllegalArgumentException("Body $i was destroyed")
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
        for (i in 0 until nextId) {
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
        for (i in 0 until nextId) {
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
        // M2: sleep states. M1: tick counter (+ optional hash, Task 6).
        tick++
    }

    private companion object {
        /** 1 ns of slack so frame times that sum to whole ticks are not lost to rounding. */
        const val ADVANCE_EPS = 1e-9
    }
}
