package io.github.itstrue54.gridironphysics

/** Player stance, set by the game. Changing it swaps the body's collider in place. */
enum class Stance {
    /** On his feet: the body's own circle collider; locomotion drives it. */
    Upright,

    /** Diving or falling: a capsule along the body angle; slides, cannot drive or turn. */
    Diving,

    /** On the ground: a capsule along the body angle with [Material.DOWNED_PLAYER]; slides to a stop. */
    Down,
}

/**
 * Physical limits of one athlete. The game layer maps ratings to these; the engine never sees ratings.
 * Immutable, so one instance can be shared. All values SI.
 */
data class AthleteParams(
    /** Top speed at effort 1 (m/s). */
    val maxSpeed: Double = 8.5,
    /** Acceleration from rest (m/s²). It tapers linearly to 0 at [maxSpeed]: a = maxAccel·(1 − v/maxSpeed). */
    val maxAccel: Double = 7.0,
    /** Braking (m/s²), constant at any speed. */
    val maxDecel: Double = 8.0,
    /** How fast the run direction and facing can turn when standing still (rad/s). */
    val turnRateAtRest: Double = 12.0,
    /** How fast the run direction can turn at [maxSpeed] (rad/s). Falls linearly from [turnRateAtRest]. */
    val turnRateAtMax: Double = 2.0,
    /** Grip coefficient: total horizontal acceleration ≤ traction × surface multiplier × g. 0.9 = cleats on dry turf. */
    val traction: Double = 0.9,
    /** 0–1. Stored for the game layer and later joints (grapple break force); M3 locomotion does not use it. */
    val balance: Double = 0.5,
) {
    init {
        require(maxSpeed.isFinite() && maxSpeed > 0.0) { "maxSpeed must be finite and > 0: $maxSpeed" }
        require(maxAccel.isFinite() && maxAccel > 0.0) { "maxAccel must be finite and > 0: $maxAccel" }
        require(maxDecel.isFinite() && maxDecel > 0.0) { "maxDecel must be finite and > 0: $maxDecel" }
        require(turnRateAtRest.isFinite() && turnRateAtRest > 0.0) { "turnRateAtRest must be finite and > 0: $turnRateAtRest" }
        require(turnRateAtMax.isFinite() && turnRateAtMax > 0.0) { "turnRateAtMax must be finite and > 0: $turnRateAtMax" }
        require(turnRateAtMax <= turnRateAtRest) { "turn rate must fall with speed: turnRateAtMax $turnRateAtMax > turnRateAtRest $turnRateAtRest" }
        require(traction.isFinite() && traction > 0.0) { "traction must be finite and > 0: $traction" }
        require(balance.isFinite() && balance in 0.0..1.0) { "balance must be in 0..1: $balance" }
    }
}

/**
 * What the game wants a player to do this tick. Mutable so the game can keep one and reuse it; [World.setIntent]
 * copies the values, so changing the object afterwards does not affect the body.
 */
class LocomotionIntent {
    enum class Mode {
        /** No drive at all: the body coasts (only damping and contacts act). */
        Idle,

        /** Run at velocity ([targetX], [targetY]) m/s, capped at effort × maxSpeed. */
        Velocity,

        /** Run to point ([targetX], [targetY]) m and stop there, at up to effort × maxSpeed. */
        Point,
    }

    var mode: Mode = Mode.Idle
        private set
    var targetX: Double = 0.0
        private set
    var targetY: Double = 0.0
        private set

    /** 0–1: the share of top speed the player is trying to reach. */
    var effort: Double = 1.0
        private set

    /** Stance to switch to along with this intent, or null to keep the current stance. */
    var stance: Stance? = null

    fun setVelocity(vx: Double, vy: Double, effort: Double = 1.0): LocomotionIntent {
        mode = Mode.Velocity; targetX = vx; targetY = vy; this.effort = effort
        return this
    }

    fun setPoint(x: Double, y: Double, effort: Double = 1.0): LocomotionIntent {
        mode = Mode.Point; targetX = x; targetY = y; this.effort = effort
        return this
    }

    fun setIdle(): LocomotionIntent {
        mode = Mode.Idle; targetX = 0.0; targetY = 0.0; effort = 1.0
        return this
    }

    /** Copies every field of [other] into this intent without allocating. Returns this. */
    fun copyFrom(other: LocomotionIntent): LocomotionIntent {
        mode = other.mode; targetX = other.targetX; targetY = other.targetY; effort = other.effort; stance = other.stance
        return this
    }

    /** Sets every field as is (snapshots and replays restore values that were valid when written). */
    internal fun setRaw(mode: Mode, targetX: Double, targetY: Double, effort: Double, stance: Stance?) {
        this.mode = mode; this.targetX = targetX; this.targetY = targetY; this.effort = effort; this.stance = stance
    }

    internal fun validate() {
        require(targetX.isFinite() && targetY.isFinite()) { "intent target must be finite: ($targetX, $targetY)" }
        require(effort.isFinite() && effort in 0.0..1.0) { "effort must be in 0..1: $effort" }
    }

    override fun toString(): String = "LocomotionIntent($mode, $targetX, $targetY, effort=$effort, stance=$stance)"

    companion object {
        fun velocity(vx: Double, vy: Double, effort: Double = 1.0): LocomotionIntent = LocomotionIntent().setVelocity(vx, vy, effort)
        fun point(x: Double, y: Double, effort: Double = 1.0): LocomotionIntent = LocomotionIntent().setPoint(x, y, effort)
    }
}

/**
 * The playing surface. Scales locomotion traction (and, from M6, ball bounce). It does not change the
 * contact [MaterialTable]. Values are tunable starting points, not measured data.
 */
class SurfaceConfig(
    val name: String,
    /** Multiplies every athlete's [AthleteParams.traction]. 1.0 = dry artificial turf. */
    val tractionMultiplier: Double,
    /** Multiplies the ball–turf restitution (used by ball bounces in M6). */
    val ballBounceMultiplier: Double,
) {
    init {
        require(tractionMultiplier.isFinite() && tractionMultiplier > 0.0) { "tractionMultiplier must be finite and > 0: $tractionMultiplier" }
        require(ballBounceMultiplier.isFinite() && ballBounceMultiplier >= 0.0) { "ballBounceMultiplier must be finite and >= 0: $ballBounceMultiplier" }
    }

    override fun toString(): String = "SurfaceConfig($name, traction×$tractionMultiplier, bounce×$ballBounceMultiplier)"

    companion object {
        val ARTIFICIAL_TURF = SurfaceConfig("Artificial turf", 1.0, 1.0)
        val NATURAL_GRASS = SurfaceConfig("Natural grass", 0.95, 0.9)
        val WET = SurfaceConfig("Wet", 0.75, 0.7)
        val FROZEN = SurfaceConfig("Frozen", 0.55, 1.1)
    }
}
