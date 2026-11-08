package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.scenarios.Scenario
import io.github.itstrue54.gridironphysics.scenarios.Scenarios

/** Plain (non-Compose) viewer state, so it can be unit-tested. The UI redraws after every frame. */
class ViewerState(scenario: Scenario = Scenarios.all.first()) {
    var scenario: Scenario = scenario
        set(value) { field = value; reset() }

    var world: World = scenario.build()
        private set

    var paused: Boolean = false

    /** Simulation speed multiplier: 0.25, 1 or 4. */
    var speed: Double = 1.0

    var selected: BodyId? = null

    /** Interpolation fraction between the previous and current tick, for drawing. */
    var alpha: Double = 0.0
        private set

    private val idBuf = IntArray(256)

    fun reset() {
        world = scenario.build()
        selected = null
        alpha = 0.0
    }

    /** Advances exactly one tick (used while paused). Drawn at the new position, not interpolated. */
    fun stepOnce() {
        world.step()
        alpha = 1.0
    }

    /** Called once per rendered frame with the real elapsed time. */
    fun frame(frameSeconds: Double) {
        if (paused) return
        alpha = world.advance(frameSeconds * speed)
    }

    /** Live body ids, ascending. The returned array is reused; read only the first [count] entries. */
    fun ids(): IntArray = idBuf
    val count: Int get() = world.bodyIds(idBuf)

    /** The body whose circle contains the world point, nearest center first, or null. */
    fun pick(wx: Double, wy: Double): BodyId? {
        var best: BodyId? = null
        var bestD2 = Double.MAX_VALUE
        val n = count
        for (i in 0 until n) {
            val b = world.body(BodyId(idBuf[i]))
            val dx = b.x - wx
            val dy = b.y - wy
            val d2 = dx * dx + dy * dy
            val r = b.radius
            if (d2 <= r * r && d2 < bestD2) {
                bestD2 = d2
                best = BodyId(idBuf[i])
            }
        }
        return best
    }
}
