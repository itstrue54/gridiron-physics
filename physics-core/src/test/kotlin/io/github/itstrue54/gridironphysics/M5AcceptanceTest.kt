package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** Spec Testing table, M5 row: Grapple break. */
class M5AcceptanceTest {
    private val ap = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)

    /** A carrier drives away from a tackler who has wrapped him up; the tackler's feet are planted (static). */
    private fun setup(breakForce: Double): Triple<World, BodyId, JointId> {
        val w = World(WorldConfig(), FieldConfig.PRO, 3L)
        val tackler = w.createBody(BodyDef(type = BodyType.Static, x = 50.0, y = 24.0, shape = ShapeDef.Circle(0.34)))
        val carrier = w.createBody(BodyDef(x = 50.7, y = 24.0, mass = 100.0, shape = ShapeDef.Circle(0.34)))
        w.setAthlete(carrier, ap)
        w.setIntent(carrier, LocomotionIntent.velocity(9.0, 0.0))
        val j = w.createGrapple(tackler, carrier, GrappleDef(breakForce = breakForce, dragForce = 100.0))
        return Triple(w, carrier, j)
    }

    /**
     * Carrier force above breakForce: the joint breaks within 1 tick and GrappleBroken is emitted. An unbreakable
     * run of the same setup shows the tension the joint would need each tick; the breakable run must break on the
     * first tick that tension goes over breakForce, and not before.
     */
    @Test fun grappleBreak() {
        val breakForce = 450.0
        val (free, _, fj) = setup(Double.POSITIVE_INFINITY)
        var firstOver = -1L
        for (t in 1..120) {
            free.step()
            if (free.joint(fj).force > breakForce) { firstOver = free.tick; break }
        }
        assertTrue(firstOver > 1, "the 700 N carrier eventually pulls harder than $breakForce N (first over at $firstOver)")

        val (w, carrier, j) = setup(breakForce)
        var broken: Ev? = null
        while (w.tick < firstOver) {
            assertTrue(w.hasJoint(j), "intact before tick $firstOver (now ${w.tick})")
            assertTrue(w.joint(j).force <= breakForce)
            w.step()
            broken = w.drain().firstOrNull { it.type == EventType.GrappleBroken } ?: broken
        }
        assertFalse(w.hasJoint(j), "broken on tick $firstOver")
        val e = broken!!
        assertEquals(firstOver, e.tick, "within 1 tick: the same tick the force first exceeded breakForce")
        assertEquals(carrier.value, e.b)
        assertEquals(j.value, e.joint)
        // After the break the carrier runs free.
        val v = w.body(carrier).vx
        repeat(10) { w.step() }
        assertTrue(w.body(carrier).vx > v)
    }
}
