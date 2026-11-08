package io.github.itstrue54.gridironphysics

/** Step and solver settings. The timestep itself is fixed at [DT] and is not configurable. */
data class WorldConfig(
    val maxBodies: Int = 64,
    val maxTicksPerFrame: Int = 4,
    val velocityIterations: Int = 8,
    val relaxIterations: Int = 3,
    val contactHertz: Double = 30.0,
    val contactDampingRatio: Double = 10.0,
    val maxPushoutSpeed: Double = 3.0,
    val sleepTimeS: Double = 0.5,
    val sleepSpeed: Double = 0.05,
    val gravity: Double = 9.81,
    val hashEveryTick: Boolean = false,
) {
    init {
        require(maxBodies > 0) { "maxBodies must be > 0" }
        require(maxTicksPerFrame > 0) { "maxTicksPerFrame must be > 0" }
        require(velocityIterations > 0) { "velocityIterations must be > 0" }
        require(relaxIterations >= 0) { "relaxIterations must be >= 0" }
        require(contactHertz > 0.0 && contactDampingRatio >= 0.0 && maxPushoutSpeed >= 0.0) { "bad contact softness" }
        require(sleepTimeS >= 0.0 && sleepSpeed >= 0.0 && gravity >= 0.0) { "bad sleep/gravity" }
    }

    companion object {
        /** The one and only tick length: 1/60 s. */
        const val DT: Double = 1.0 / 60.0
    }
}
