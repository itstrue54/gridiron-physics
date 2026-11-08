package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** The spec's Testing-table rows gated by M3: Top speed & acceleration, Traction limit. */
class M3AcceptanceTest {

    private fun speed(w: World, id: BodyId): Double = w.body(id).let { DMath.hypot2(it.vx, it.vy) }

    /**
     * Effort 1 from rest. Design decision 4: acceleration tapers as a = maxAccel·(1 − v/maxSpeed), so
     * v(t) = maxSpeed·(1 − e^(−t/τ)) with τ = maxSpeed/maxAccel. Top speed is checked after 5τ (±1%), and the
     * time to 90% against t90 = τ·ln 10 (±2%).
     */
    @Test fun topSpeedAndAcceleration() {
        val athletes = listOf(
            AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0),
            AthleteParams(maxSpeed = 7.0, maxAccel = 5.0, maxDecel = 7.0, turnRateAtRest = 10.0, turnRateAtMax = 3.0),
        )
        for (ap in athletes) {
            val w = World(WorldConfig(), FieldConfig.PRO, 1L)
            val p = w.createBody(BodyDef(x = 5.0, y = 24.0, mass = 95.0, shape = ShapeDef.Circle(0.33)))
            w.setAthlete(p, ap)
            w.setIntent(p, LocomotionIntent.velocity(ap.maxSpeed, 0.0, effort = 1.0))

            val tau = ap.maxSpeed / ap.maxAccel
            val t90Analytic = tau * DMath.log(10.0)
            val ticks = kotlin.math.ceil(5.0 * tau / WorldConfig.DT).toInt()
            var t90 = Double.NaN
            for (t in 1..ticks) {
                w.step()
                val v = speed(w, p)
                assertTrue(v <= ap.maxSpeed * (1 + 1e-9), "never faster than maxSpeed: $v")
                if (t90.isNaN() && v >= 0.9 * ap.maxSpeed) t90 = t * WorldConfig.DT
            }
            assertEquals(t90Analytic, t90, 0.02 * t90Analytic, "time to 90% of ${ap.maxSpeed} m/s")
            assertEquals(ap.maxSpeed, speed(w, p), 0.01 * ap.maxSpeed, "speed after 5τ")
            assertEquals(0.0, w.body(p).vy, 1e-12, "runs straight")
        }
    }

    /**
     * A hard 90° cut at top speed. The horizontal acceleration never exceeds traction × surface × g, and the
     * clamped intent reports a Slip with the excess. Checked on dry turf (the spec's traction × g) and on a wet
     * surface, where the limit is lower.
     */
    @Test fun tractionLimit() {
        val ap = AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)
        for (surface in listOf(SurfaceConfig.ARTIFICIAL_TURF, SurfaceConfig.WET)) {
            val w = World(WorldConfig(), FieldConfig.PRO, 1L)
            w.setSurface(surface)
            val p = w.createBody(BodyDef(x = 20.0, y = 10.0, vx = ap.maxSpeed, mass = 95.0, shape = ShapeDef.Circle(0.33)))
            w.setAthlete(p, ap)
            w.setIntent(p, LocomotionIntent.velocity(ap.maxSpeed, 0.0))
            val straight = w.run(30)
            assertEquals(ap.maxSpeed, speed(w, p), 1e-9, "steady at top speed before the cut")
            assertTrue(straight.none { it.type == EventType.Slip }, "no slip running straight")

            w.setIntent(p, LocomotionIntent.velocity(0.0, ap.maxSpeed)) // cut 90° to the left
            val limit = ap.traction * surface.tractionMultiplier * w.config.gravity
            var maxA = 0.0
            var slips = 0
            repeat(240) {
                val vx0 = w.body(p).vx; val vy0 = w.body(p).vy
                w.step()
                val a = DMath.hypot2(w.body(p).vx - vx0, w.body(p).vy - vy0) / WorldConfig.DT
                assertTrue(a <= limit * (1 + 1e-9), "${surface.name}: horizontal acceleration $a exceeds traction limit $limit")
                if (a > maxA) maxA = a
                for (e in w.drain()) {
                    if (e.type == EventType.Slip && e.a == p.value) {
                        assertTrue(e.value > 0.0, "slip excess is positive")
                        slips++
                    }
                }
            }
            assertTrue(slips > 0, "${surface.name}: the hard cut must report Slip")
            assertTrue(maxA > 0.99 * limit, "${surface.name}: the cut uses all the traction there is ($maxA of $limit)")
            val b = w.body(p)
            assertTrue(b.vy > 0.99 * DMath.hypot2(b.vx, b.vy) && b.vy > 3.0, "${surface.name}: the cut completes: (${b.vx}, ${b.vy})")
        }
    }
}
