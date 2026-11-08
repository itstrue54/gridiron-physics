package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.AthleteParams
import io.github.itstrue54.gridironphysics.LocomotionIntent
import io.github.itstrue54.gridironphysics.Stance
import io.github.itstrue54.gridironphysics.WorldConfig
import io.github.itstrue54.gridironphysics.math.DMath

/**
 * Turns one player's intent into a force each tick (spec §Locomotion controller). Holds the athlete's limits and
 * a copy of the current intent. All math is per tick at the fixed 1/60 s step and never allocates.
 *
 * Upright, the controller picks a desired velocity from the intent, then moves the current velocity toward it:
 * - the run direction turns at most turnRate(speed)·dt per tick, falling linearly from turnRateAtRest to turnRateAtMax;
 * - speed rises along the taper a = maxAccel·(1 − v/maxSpeed) (design decision 4), integrated exactly per tick
 *   so the sprint curve is v(t) = maxSpeed·(1 − e^(−t/τ)); speed falls at maxDecel without taper;
 * - while the remaining turn is large, the target speed shrinks by its cosine, so a player plants and cuts
 *   instead of running a wide circle;
 * - the total horizontal acceleration is clamped to the traction limit, and the clamped part is the Slip excess.
 * The player faces his desired direction, turning at the same speed-dependent rate.
 *
 * Diving and Down players cannot drive or turn: they slide, braked by ground friction (WorldConfig.slideFriction).
 */
internal class LocomotionController(params: AthleteParams) {
    @JvmField val intent = LocomotionIntent()

    var params: AthleteParams = params
        set(value) {
            field = value
            stepGain = 1.0 - DMath.exp(-WorldConfig.DT * value.maxAccel / value.maxSpeed)
        }

    /** Share of the gap to maxSpeed closed in one tick of tapered acceleration: 1 − e^(−dt/τ). */
    private var stepGain = 1.0 - DMath.exp(-WorldConfig.DT * params.maxAccel / params.maxSpeed)

    /** Acceleration (m/s²) the intent asked for beyond the traction limit on the last tick; 0 when none. */
    @JvmField var slip = 0.0

    /** False when the intent is satisfied by standing still, so the body may fall asleep. */
    @JvmField var wantsMotion = false

    /**
     * Sets [Body.fx], [Body.fy] (and, upright, the angular velocity) for this tick. [grip] = traction·surface·g is
     * the cleats' limit; [slideGrip] = slideFriction·surface·g brakes a Diving or Down player.
     */
    fun drive(b: Body, grip: Double, slideGrip: Double) {
        slip = 0.0
        val dt = WorldConfig.DT
        val vx = b.vx
        val vy = b.vy
        val s = DMath.hypot2(vx, vy)
        if (b.stance != Stance.Upright) {
            // Sliding on the ground: ground friction brakes him, no steering.
            wantsMotion = s > 0.0
            if (s == 0.0) return
            val dv = DMath.min(slideGrip * dt, s)
            b.fx = -b.mass * vx / s * dv / dt
            b.fy = -b.mass * vy / s * dv / dt
            return
        }
        val it = intent
        if (it.mode == LocomotionIntent.Mode.Idle) {
            wantsMotion = false
            return
        }
        val p = params
        val vcap = it.effort * p.maxSpeed

        // Desired direction (dx, dy) and speed sd.
        var dx = 0.0
        var dy = 0.0
        var sd: Double
        if (it.mode == LocomotionIntent.Mode.Velocity) {
            val m = DMath.hypot2(it.targetX, it.targetY)
            sd = DMath.min(m, vcap)
            if (m > 0.0) { dx = it.targetX / m; dy = it.targetY / m }
        } else {
            val ex = it.targetX - b.x
            val ey = it.targetY - b.y
            val d = DMath.hypot2(ex, ey)
            if (d <= ARRIVE_RADIUS) {
                sd = 0.0
            } else {
                dx = ex / d; dy = ey / d
                // Arrive: slow early enough to stop at the point, and never step past it in one tick.
                sd = DMath.min(vcap, DMath.min(DMath.sqrt(2.0 * p.maxDecel * (d - ARRIVE_RADIUS)), (d - ARRIVE_RADIUS) / dt))
            }
        }
        if (sd <= 0.0) sd = 0.0
        wantsMotion = sd > 0.0 || s > 0.0

        val turnRate = p.turnRateAtRest + (p.turnRateAtMax - p.turnRateAtRest) * DMath.min(s / p.maxSpeed, 1.0)
        val accelStep = DMath.max(p.maxSpeed - s, 0.0) * stepGain
        val decelStep = p.maxDecel * dt

        var nvx: Double
        var nvy: Double
        if (s < STEER_SPEED) {
            // Near rest the run direction is undefined: steer straight at the desired velocity.
            val wx = dx * sd - vx
            val wy = dy * sd - vy
            val w = DMath.hypot2(wx, wy)
            val lim = if (sd >= s) accelStep else decelStep
            val k = if (w > lim) lim / w else 1.0
            nvx = vx + wx * k
            nvy = vy + wy * k
        } else {
            val hv = DMath.atan2(vy, vx)
            var turn = 0.0
            var target = 0.0
            if (sd > 0.0) {
                val err = wrap(DMath.atan2(dy, dx) - hv)
                val maxTurn = turnRate * dt
                turn = DMath.clamp(err, -maxTurn, maxTurn)
                target = sd * DMath.max(DMath.cos(err - turn), 0.0)
            }
            val ns = if (target >= s) s + DMath.min(accelStep, target - s) else s - DMath.min(decelStep, s - target)
            val h = hv + turn
            nvx = ns * DMath.cos(h)
            nvy = ns * DMath.sin(h)
        }

        // Traction: total horizontal acceleration ≤ grip; the rest is the Slip excess.
        var ax = (nvx - vx) / dt
        var ay = (nvy - vy) / dt
        val a = DMath.hypot2(ax, ay)
        if (a > grip) {
            slip = a - grip
            ax *= grip / a
            ay *= grip / a
        }
        b.fx = b.mass * ax
        b.fy = b.mass * ay

        // Facing: toward where he wants to go; while stopping, along his motion; at rest, unchanged.
        val face = if (sd > 0.0) DMath.atan2(dy, dx) else if (s > STEER_SPEED) DMath.atan2(vy, vx) else b.angle
        b.angularVelocity = DMath.clamp(wrap(face - b.angle) / dt, -turnRate, turnRate)
    }

    private companion object {
        /** Below this speed (m/s) the controller steers in x/y instead of turning a heading. */
        const val STEER_SPEED = 0.5

        /** A point intent counts as reached inside this distance (m). */
        const val ARRIVE_RADIUS = 0.01

        fun wrap(a: Double): Double = DMath.wrapPi(a)
    }
}
