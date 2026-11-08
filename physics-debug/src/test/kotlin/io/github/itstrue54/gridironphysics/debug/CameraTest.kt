package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.FieldConfig
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** The viewer camera: follow the action by default, zoom with the wheel, pan by dragging, full field on demand. */
class CameraTest {
    private val f = FieldConfig.PRO

    @Test fun transformFitsAndCentersAViewRect() {
        val t = ViewTransform(f, 1000f, 1000f, view = ViewRect(50.0, 20.0, 60.0, 30.0))
        assertEquals((1000f - 32f) / 10f, t.pxPerMeter, 1e-3f)
        assertEquals(500f, t.toScreenX(55.0), 0.01f)
        assertEquals(500f, t.toScreenY(25.0), 0.01f)
        assertEquals(57.5, t.toWorldX(t.toScreenX(57.5)), 1e-3)
        assertEquals(21.25, t.toWorldY(t.toScreenY(21.25)), 1e-3)
    }

    @Test fun defaultViewIsTheWholeWorld() {
        val t = ViewTransform(f, 1280f, 720f)
        val full = ViewTransform(f, 1280f, 720f, view = ViewRect.wholeWorld(f))
        assertEquals(full.pxPerMeter, t.pxPerMeter)
        assertEquals(full.toScreenX(0.0), t.toScreenX(0.0))
    }

    private fun dynamicBodies(s: ViewerState): List<BodyId> {
        val n = s.count
        return (0 until n).map { BodyId(s.ids()[it]) }.filter { s.world.body(it).type != BodyType.Static }
    }

    private fun assertContainsBodies(s: ViewerState, r: ViewRect) {
        dynamicBodies(s).forEach {
            val b = s.world.body(it)
            assertTrue(b.x - b.radius >= r.minX && b.x + b.radius <= r.maxX && b.y - b.radius >= r.minY && b.y + b.radius <= r.maxY, "body ${it.value} at (${b.x}, ${b.y}) outside $r")
        }
    }

    @Test fun followsTheActionByDefaultAndPlayersAreBigEnoughToSee() {
        val s = ViewerState(Scenarios.headOnTackle)
        assertTrue(s.follow)
        val r = s.viewRect()
        assertContainsBodies(s, r)
        assertTrue(r.width <= 30.0, "tackle framing should be tight, was ${r.width} m wide")
        val t = ViewTransform(f, 1200f, 800f, view = r)
        assertTrue(t.len(0.32) >= 10f, "a 0.32 m player must be at least 10 px, was ${t.len(0.32)}")
    }

    @Test fun followKeepsTrackingAsBodiesMove() {
        val s = ViewerState(Scenarios.contactLab)
        repeat(120) { s.stepOnce() }
        assertContainsBodies(s, s.viewRect())
    }

    @Test fun followNeverZoomsInPastAMinimumWindow() {
        val s = ViewerState(Scenarios.headOnTackle)
        repeat(36) { s.stepOnce() } // players meet: bodies nearly touching
        assertTrue(s.viewRect().width >= ViewerState.MIN_VIEW_M - 1e-9)
        assertTrue(s.viewRect().height >= ViewerState.MIN_VIEW_M - 1e-9)
    }

    @Test fun wheelZoomKeepsTheCursorPointFixedAndStopsFollowing() {
        val s = ViewerState(Scenarios.headOnTackle)
        val before = s.viewRect()
        val px = before.minX + before.width * 0.25
        val py = before.minY + before.height * 0.75
        s.zoomBy(2.0, px, py)
        val after = s.viewRect()
        assertFalse(s.follow)
        assertEquals(before.width / 2, after.width, 1e-9)
        assertEquals(0.25, (px - after.minX) / after.width, 1e-9)
        assertEquals(0.75, (py - after.minY) / after.height, 1e-9)
    }

    @Test fun dragPansAndStopsFollowing() {
        val s = ViewerState(Scenarios.headOnTackle)
        val before = s.viewRect()
        s.panBy(3.0, -2.0)
        val after = s.viewRect()
        assertFalse(s.follow)
        assertEquals(before.minX + 3.0, after.minX, 1e-9)
        assertEquals(before.minY - 2.0, after.minY, 1e-9)
    }

    @Test fun fullFieldShowsTheWholeWorldAndFollowComesBack() {
        val s = ViewerState(Scenarios.headOnTackle)
        s.showFullField()
        assertFalse(s.follow)
        assertEquals(ViewRect.wholeWorld(f), s.viewRect())
        s.follow = true
        assertTrue(s.viewRect().width <= 30.0)
    }

    @Test fun changingScenarioRestoresFollow() {
        val s = ViewerState(Scenarios.headOnTackle)
        s.showFullField()
        s.scenario = Scenarios.contactLab
        assertTrue(s.follow)
    }
}

class PlayerLookTest {
    @Test fun playersDrawAsFiguresByDefaultWithCollidersOnDemand() {
        val s = ViewerState()
        assertFalse(s.showColliders)
        s.showColliders = true
        s.reset()
        assertTrue(s.showColliders, "the collider toggle is a view preference; reset keeps it")
    }

    @Test fun onlyPlayersAndOfficialsGetFigures() {
        assertTrue(isFigure(io.github.itstrue54.gridironphysics.scenarios.Tags.HOME))
        assertTrue(isFigure(io.github.itstrue54.gridironphysics.scenarios.Tags.AWAY))
        assertTrue(isFigure(io.github.itstrue54.gridironphysics.scenarios.Tags.OFFICIAL))
        assertFalse(isFigure(io.github.itstrue54.gridironphysics.scenarios.Tags.PROP))
    }
}

class WindowTest {
    @Test fun viewerStaysOnTopByDefaultAndCanBeUnpinned() {
        val s = ViewerState()
        assertTrue(s.alwaysOnTop, "a Gradle-launched window can't take focus on Windows, so it pins itself on top")
        s.alwaysOnTop = false
        s.reset()
        assertFalse(s.alwaysOnTop, "a window preference; reset keeps it")
    }
}
