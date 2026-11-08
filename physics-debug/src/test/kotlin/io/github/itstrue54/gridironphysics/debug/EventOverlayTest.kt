package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.EventType
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** The viewer drains the world's events every frame and keeps short-lived markers and a log to draw. */
class EventOverlayTest {
    private fun run(s: ViewerState, seconds: Double) = repeat((seconds * 60).toInt()) { s.frame(1.0 / 60) }

    @Test fun eventLabProducesMarkersForHitsTripsAndLines() {
        val s = ViewerState(Scenarios.eventLab)
        val seen = HashSet<EventType>()
        repeat(180) { s.frame(1.0 / 60); s.markers.forEach { seen += it.type } }
        for (k in listOf(EventType.Hit, EventType.Trip, EventType.Slip, EventType.OutOfBounds, EventType.GoalLineCrossed, EventType.SensorBegin)) {
            assertTrue(k in seen, "no $k marker: $seen")
        }
        assertEquals(0, s.world.events().pending, "the viewer drains the stream")
    }

    @Test fun markersFadeAndTheLogIsBounded() {
        val s = ViewerState(Scenarios.eventLab)
        run(s, 3.0)
        assertTrue(s.markers.all { s.world.tick - it.tick < ViewerState.MARKER_TICKS }, "old markers are dropped")
        assertTrue(s.eventLog.isNotEmpty() && s.eventLog.size <= ViewerState.LOG_SIZE)
        s.reset()
        assertTrue(s.markers.isEmpty() && s.eventLog.isEmpty(), "reset clears the overlay")
    }

    @Test fun contactBeginAndEndAreNotMarked() {
        val s = ViewerState(Scenarios.eventLab)
        run(s, 3.0)
        assertTrue(s.markers.none { it.type == EventType.ContactBegin || it.type == EventType.ContactEnd }, "too many to draw")
    }

    @Test fun steppingWhilePausedAlsoCollectsEvents() {
        val s = ViewerState(Scenarios.eventLab)
        s.paused = true
        repeat(60) { s.stepOnce() }
        assertTrue(s.eventLog.isNotEmpty())
    }
}
