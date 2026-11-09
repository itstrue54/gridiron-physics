package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** M8: every scenario runs at 4x (four ticks per 60 Hz frame), and the top bar can show what a step costs. */
class SpeedTest {
    @Test fun everyScenarioRunsFourTicksPerFrameAt4x() {
        for (sc in Scenarios.all) {
            val s = ViewerState(sc)
            s.speed = 4.0
            repeat(60) { s.frame(1.0 / 60) }
            assertEquals(240L, s.world.tick, "${sc.name}: one second at 4x is 240 ticks")
            assertTrue(s.ticksPerFrame == 4, "${sc.name}: ${s.ticksPerFrame} ticks in the last frame")
        }
    }

    @Test fun stepTimeIsMeasuredAndResetClearsIt() {
        val s = ViewerState(Scenarios.scrimmage)
        assertEquals(0.0, s.stepMicros)
        s.speed = 4.0
        repeat(30) { s.frame(1.0 / 60) }
        assertTrue(s.stepMicros > 0.0 && s.stepMicros < 100_000.0, "step time ${s.stepMicros} us")
        s.reset()
        assertEquals(0.0, s.stepMicros)
        assertEquals(0, s.ticksPerFrame)
    }

    @Test fun theScrimmageIsTheFirstScenario() {
        assertEquals(Scenarios.scrimmage, ViewerState().scenario)
    }
}
