package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BallRecord

/** How the ball flies; it sets the drag coefficient ([BallConfig.cd]). */
enum class FlightMode {
    /** A tight spiral: lowest drag. */
    Spiral,

    /** A wobbling pass or a shanked punt. */
    Wobble,

    /** A tumbling kick: highest drag. */
    EndOverEnd,
}

/**
 * Where the ball is. [Dead] → [Airborne] ([World.launchBall]) ↔ [Loose] (it lands and rolls as a 2D body; a
 * [World.deflect] upward or a new launch sends it back up) → [Carried] ([World.setBallCarrier]). Taking the ball away
 * from its carrier with `setBallCarrier(null)` makes it Dead again.
 */
enum class BallState { Dead, Airborne, Loose, Carried }

/**
 * Where a ball in the air met an end-line plane ([EventType.KickResult]). Left and right are the kicker's, facing the
 * way the ball travels. The engine reports what the ball did; whether it was a kick, and whether it counts, is the game's call.
 */
enum class KickResult {
    /** Between the uprights (or their extended lines) and above the crossbar. */
    Good,
    WideLeft,
    WideRight,

    /** Between the uprights but under the crossbar. */
    Short,

    /** The ball touched an upright, the crossbar or a gooseneck. Reported at the touch; the ball bounces off. */
    HitPost,
}

/**
 * The ball's physical constants. All tunable starting values, not measured data. The cross-section is the
 * nose-on area of a regulation ball; [cd] carries the difference between flight modes. The defaults cost a 27 m/s
 * kick about a fifth of its vacuum range end over end, so a strong leg makes a 50-yard field goal.
 */
data class BallConfig(
    val mass: Double = 0.41,
    /** Ground collider radius, and the sphere used against the goal posts in the air (m). */
    val radius: Double = 0.09,
    /** Cross-section A in the drag law (m²). */
    val crossSection: Double = 0.0227,
    val airDensity: Double = 1.225,
    val cdSpiral: Double = 0.06,
    val cdWobble: Double = 0.12,
    val cdEndOverEnd: Double = 0.18,
    /** False flies the ball in vacuum (no drag, so no wind either). */
    val drag: Boolean = true,
    /** A bounce turns the ball's ground direction by a seeded random angle within ±this (rad); 35°. */
    val bounceYawMax: Double = 35.0 * 3.141592653589793 / 180.0,
    /** After a bounce slower than this upward (m/s), the ball stays on the ground and rolls as a loose body. */
    val minBounceSpeed: Double = 1.0,
    /** A loose ball slows by rollFriction × g (m/s²) until it stops. */
    val rollFriction: Double = 0.35,
    /** The lowest a ball can be caught (m); the top is each player's reach height ([World.setReach]). */
    val catchMinHeight: Double = 0.3,
    /** Height of a carried ball (m), for [BallView.z]. */
    val carryHeight: Double = 1.0,
) {
    init {
        require(mass.isFinite() && mass > 0.0 && radius.isFinite() && radius > 0.0) { "ball mass and radius must be > 0" }
        require(crossSection.isFinite() && crossSection > 0.0 && airDensity.isFinite() && airDensity >= 0.0) { "bad ball aerodynamics" }
        require(cdSpiral >= 0.0 && cdWobble >= 0.0 && cdEndOverEnd >= 0.0 && cdSpiral.isFinite() && cdWobble.isFinite() && cdEndOverEnd.isFinite()) { "drag coefficients must be finite and >= 0" }
        require(bounceYawMax.isFinite() && bounceYawMax in 0.0..3.141592653589793) { "bounceYawMax must be in 0..π: $bounceYawMax" }
        require(minBounceSpeed.isFinite() && minBounceSpeed >= 0.0 && rollFriction.isFinite() && rollFriction >= 0.0) { "bad bounce/roll settings" }
        require(catchMinHeight.isFinite() && catchMinHeight >= 0.0 && carryHeight.isFinite() && carryHeight >= 0.0) { "bad catch/carry heights" }
    }

    fun cd(mode: FlightMode): Double = when (mode) {
        FlightMode.Spiral -> cdSpiral
        FlightMode.Wobble -> cdWobble
        FlightMode.EndOverEnd -> cdEndOverEnd
    }
}

/**
 * Read-only view of the ball, from [World.ball]. World reuses one instance, which always shows the current ball.
 * Positions are in meters with z the ball's height above the turf (0 on the ground). While [state] is
 * [BallState.Loose] the ball is also the body [World.ballId] and these values mirror it; while Carried they follow
 * the carrier. A Dead ball stays where it was.
 */
class BallView internal constructor(private val s: BallRecord) {
    val state: BallState get() = BallState.entries[s.mode]
    val flightMode: FlightMode get() = FlightMode.entries[s.flightMode]
    val x: Double get() = s.x
    val y: Double get() = s.y
    val z: Double get() = s.z
    val vx: Double get() = s.vx
    val vy: Double get() = s.vy
    val vz: Double get() = s.vz

    /** Position before the most recent step, for interpolated drawing. */
    val prevX: Double get() = s.prevX
    val prevY: Double get() = s.prevY
    val prevZ: Double get() = s.prevZ
}
