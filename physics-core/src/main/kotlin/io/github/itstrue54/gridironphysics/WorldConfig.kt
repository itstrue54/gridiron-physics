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
    /** Closing speed (m/s) above which restitution applies, so players in contact don't jitter. */
    val restitutionThreshold: Double = 1.0,
    val materials: MaterialTable = MaterialTable.DEFAULT,
    /** Static walls at the edge of the runoff that stop bodies leaving the world. */
    val runoffWalls: Boolean = true,
    /** Contact pool size. Pairs beyond it are dropped (in id order) for that tick, and report ContactEnd if touching. */
    val maxContacts: Int = 512,
    /**
     * Friction coefficient of a Diving or Down player sliding on the ground, scaled by the surface's traction
     * multiplier. It is the body's, not the cleats', so it does not use the athlete's traction.
     */
    val slideFriction: Double = 0.8,
    /** Size of the event ring buffer ([World.events]). When the game falls behind, the oldest events are lost. */
    val eventCapacity: Int = 1024,
    /** Slowest a player can move (m/s) and still trip over a Down body ([EventType.Trip]). */
    val tripMinSpeed: Double = 1.0,
    /** Joint pool size ([World.createGrapple] and friends throw when it is full). */
    val maxJoints: Int = 64,
    /** Softness of rigid joint rows (distance and weld with hertz 0) while solving velocities, as in Box2D v3. */
    val jointHertz: Double = 60.0,
    val jointDampingRatio: Double = 2.0,
    /** The ball: mass, size, drag per flight mode, bounces, catch and carry heights. */
    val ball: BallConfig = BallConfig(),
    /**
     * The two goal-post assemblies ([FieldConfig]): 3D uprights, crossbar and gooseneck for the ball in the air, plus a
     * padded base behind each end line that players and the loose ball collide with.
     */
    val goalPosts: Boolean = true,
) {
    init {
        require(maxBodies > 0) { "maxBodies must be > 0" }
        require(maxTicksPerFrame > 0) { "maxTicksPerFrame must be > 0" }
        require(velocityIterations > 0) { "velocityIterations must be > 0" }
        require(relaxIterations >= 0) { "relaxIterations must be >= 0" }
        require(contactHertz > 0.0 && contactDampingRatio >= 0.0 && maxPushoutSpeed >= 0.0) { "bad contact softness" }
        require(restitutionThreshold >= 0.0 && maxContacts > 0) { "bad restitutionThreshold/maxContacts" }
        require(slideFriction.isFinite() && slideFriction >= 0.0) { "slideFriction must be finite and >= 0: $slideFriction" }
        require(eventCapacity > 0) { "eventCapacity must be > 0: $eventCapacity" }
        require(tripMinSpeed.isFinite() && tripMinSpeed >= 0.0) { "tripMinSpeed must be finite and >= 0: $tripMinSpeed" }
        require(maxJoints in 1..BodyId.MAX_SLOTS) { "maxJoints must be in 1..${BodyId.MAX_SLOTS}: $maxJoints" }
        require(jointHertz.isFinite() && jointHertz > 0.0 && jointDampingRatio.isFinite() && jointDampingRatio >= 0.0) { "bad joint softness" }
        require(sleepTimeS >= 0.0 && sleepSpeed >= 0.0 && gravity >= 0.0) { "bad sleep/gravity" }
    }

    companion object {
        /** The one and only tick length: 1/60 s. */
        const val DT: Double = 1.0 / 60.0
    }
}
