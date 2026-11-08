package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.BallState
import io.github.itstrue54.gridironphysics.EventType
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** The viewer's ball: a shadow on the ground, the ball lifted by its height with a label, a trail, and the ball events. */
class BallViewTest {
    private fun run(s: ViewerState, ticks: Int, each: () -> Unit = {}) = repeat(ticks) { s.frame(1.0 / 60); each() }

    @Test fun ballScenariosMarkTheirBallEvents() {
        for ((sc, kind) in listOf(
            Scenarios.deepPass to EventType.CatchOpportunity,
            Scenarios.punt to EventType.BallBounce,
            Scenarios.fieldGoal to EventType.KickResult,
        )) {
            val s = ViewerState(sc)
            val seen = HashSet<EventType>()
            run(s, 360) { s.markers.forEach { seen += it.type } }
            assertTrue(kind in seen, "${sc.name}: no $kind marker in $seen")
        }
    }

    @Test fun kickResultsAreLoggedByName() {
        val s = ViewerState(Scenarios.fieldGoal)
        run(s, 240)
        assertTrue(s.eventLog.any { "HitPost" in it }, "log: ${s.eventLog}")
    }

    @Test fun theTrailFollowsTheBallInTheAirAndResetClearsIt() {
        val s = ViewerState(Scenarios.deepPass)
        run(s, 60)
        assertEquals(BallState.Airborne, s.world.ball().state)
        assertTrue(s.ballTrail.size > 20, "trail ${s.ballTrail.size}")
        val last = s.ballTrail.last()
        assertEquals(s.world.ball().x, last.x, 1e-9); assertEquals(s.world.ball().z, last.z, 1e-9)
        s.reset()
        assertTrue(s.ballTrail.isEmpty())
    }

    @Test fun followFramesTheBallToo() {
        val s = ViewerState(Scenarios.fieldGoal)
        repeat(100) {
            s.frame(1.0 / 60)
            val b = s.world.ball()
            val r = s.viewRect()
            assertTrue(b.x in r.minX..r.maxX && b.y in r.minY..r.maxY, "ball (${b.x}, ${b.y}) outside $r at tick ${s.world.tick}")
        }
    }

    @Test fun clickingTheLooseBallSelectsNothingSoTheInspectorNeverLosesItsBody() {
        val s = ViewerState(Scenarios.punt)
        while (s.world.ball().state != BallState.Loose && s.world.tick < 900) s.stepOnce()
        assertEquals(BallState.Loose, s.world.ball().state)
        assertEquals(null, s.pick(s.world.ball().x, s.world.ball().y))
    }

    @Test fun ballScreenOffsetGrowsWithHeight() {
        assertEquals(0f, ballLift(0.0, 10f))
        assertTrue(ballLift(10.0, 10f) > ballLift(2.0, 10f))
    }
}
