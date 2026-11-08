package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

class ForwardProgressTest {
    private val f = FieldConfig.PRO
    private val ap = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)

    private fun world() = World(WorldConfig(), f, 1L)

    private fun runner(w: World, x: Double, vx: Double = 0.0): BodyId {
        val id = w.createBody(BodyDef(x = x, y = 24.0, vx = vx, mass = 95.0, shape = ShapeDef.Circle(0.33)))
        w.setAthlete(id, ap)
        return id
    }

    private fun downs(evs: List<Ev>) = evs.filter { it.type == EventType.Down }

    @Test fun downReportsTheFarthestPointReachedTowardTheAttackDirection() {
        for (dir in listOf(1, -1)) {
            val w = world()
            val c = runner(w, 50.0)
            w.setAttackDirection(dir)
            w.setBallCarrier(c)
            assertEquals(50.0, w.forwardProgressX)
            w.setIntent(c, LocomotionIntent.velocity(dir * 8.0, 0.0))
            var best = 50.0
            fun track() { w.step(); if (dir * w.body(c).x > dir * best) best = w.body(c).x }
            repeat(90) { track() }
            assertTrue(dir * (best - 50.0) > 5.0)
            // Driven back after the farthest point: progress keeps the best spot.
            w.setIntent(c, LocomotionIntent.velocity(-dir * 8.0, 0.0))
            repeat(150) { track() }
            assertTrue(dir * (w.body(c).x - best) < -2.0)
            w.drain()
            w.setStance(c, Stance.Down)
            val d = downs(w.drain()).single()
            assertEquals(c.value, d.a)
            assertTrue(d.ballCarrier)
            assertEquals(w.tick, d.tick, "a stance change between steps is stamped with the current tick")
            assertEquals(best, d.value, "progress is the farthest end-of-tick position (dir $dir)")
            assertEquals(d.value, w.forwardProgressX)
        }
    }

    @Test fun progressFreezesWhileTheCarrierIsNotUpright() {
        val w = world()
        val c = runner(w, 50.0, vx = 8.0)
        w.setBallCarrier(c) // attack direction defaults to +1
        repeat(10) { w.step() }
        w.setStance(c, Stance.Diving)
        val atDive = w.forwardProgressX
        assertEquals(w.body(c).x, atDive, 1e-12)
        repeat(30) { w.step() }
        assertTrue(w.body(c).x > atDive + 1.0, "the dive slides on")
        assertEquals(atDive, w.forwardProgressX, "but a dive gains no progress")
        w.setIntent(c, LocomotionIntent.velocity(8.0, 0.0).also { it.stance = Stance.Down })
        val d = downs(w.drain()).single()
        assertEquals(atDive, d.value)
    }

    @Test fun downFiresOnlyWhenTheStanceBecomesDown() {
        val w = world()
        val c = runner(w, 50.0)
        val other = runner(w, 60.0)
        w.setBallCarrier(c)
        w.setStance(other, Stance.Diving)
        w.setStance(other, Stance.Down)
        w.setStance(other, Stance.Down)
        val evs = downs(w.drain())
        assertEquals(1, evs.size, "Diving is not Down, and Down to Down is no change")
        assertEquals(other.value, evs.single().a)
        assertFalse(evs.single().ballCarrier)
        assertEquals(0.0, evs.single().value, "non-carriers carry no progress")
        w.setStance(other, Stance.Upright)
        w.setStance(other, Stance.Down)
        assertEquals(1, downs(w.drain()).size, "getting up and going down again is a new Down")
    }

    @Test fun noCarrierMeansNoProgress() {
        val w = world()
        val c = runner(w, 50.0)
        assertNull(w.ballCarrier)
        assertTrue(w.forwardProgressX.isNaN())
        w.setBallCarrier(c)
        assertEquals(c, w.ballCarrier)
        w.setBallCarrier(null)
        assertTrue(w.forwardProgressX.isNaN())
        w.setStance(c, Stance.Down)
        assertFalse(downs(w.drain()).single().ballCarrier)
    }

    @Test fun aNewCarrierStartsFromWhereHeIs() {
        val w = world()
        val a = runner(w, 50.0, vx = 8.0)
        val b = runner(w, 40.0)
        w.setBallCarrier(a)
        repeat(30) { w.step() }
        w.setBallCarrier(b) // a handoff backward
        assertEquals(40.0, w.forwardProgressX)
    }

    @Test fun destroyingTheCarrierClearsIt() {
        val w = world()
        val c = runner(w, 50.0)
        w.setBallCarrier(c)
        w.destroyBody(c)
        assertNull(w.ballCarrier)
        assertTrue(w.forwardProgressX.isNaN())
    }

    @Test fun rejectsBadCarrierAndDirection() {
        val w = world()
        assertFailsWith<IllegalArgumentException> { w.setAttackDirection(0) }
        assertFailsWith<IllegalArgumentException> { w.setAttackDirection(2) }
        assertFailsWith<IllegalArgumentException> { w.setBallCarrier(BodyId(7 shl 20)) }
        val s = w.createBody(BodyDef(type = BodyType.Static, x = 10.0, y = 10.0))
        assertFailsWith<IllegalArgumentException> { w.setBallCarrier(s) }
        assertEquals(1, w.attackDirection)
    }

    @Test fun changingAttackDirectionRestartsTrackingFromTheCurrentSpot() {
        val w = world()
        val c = runner(w, 50.0, vx = 8.0)
        w.setBallCarrier(c)
        repeat(30) { w.step() }
        w.setAttackDirection(-1)
        assertEquals(w.body(c).x, w.forwardProgressX, 1e-12)
        repeat(10) { w.step() }
        assertTrue(w.forwardProgressX < w.body(c).x, "running away from the new attack direction gains nothing")
    }
}
