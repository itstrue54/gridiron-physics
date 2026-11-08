package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.FieldConfig
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class ViewTransformTest {
    private val t = ViewTransform(FieldConfig.PRO, canvasW = 1280f, canvasH = 720f)

    @Test fun originIsBottomLeftOfFieldInsideRunoff() {
        assertEquals(16f + 6f * t.pxPerMeter, t.toScreenX(0.0), 0.01f)
        assertTrue(t.toScreenY(0.0) > t.toScreenY(FieldConfig.PRO.widthM), "near sideline is at the bottom")
    }

    @Test fun roundTrip() {
        assertEquals(37.5, t.toWorldX(t.toScreenX(37.5)), 1e-3)
        assertEquals(12.25, t.toWorldY(t.toScreenY(12.25)), 1e-3)
    }

    @Test fun wholeWorldFits() {
        assertTrue(t.toScreenX(-6.0) >= 0f && t.toScreenX(109.728 + 6.0) <= 1280f)
        assertTrue(t.toScreenY(-6.0) <= 720f && t.toScreenY(48.768 + 6.0) >= 0f)
    }

    @Test fun tallCanvasStillFits() {
        val tall = ViewTransform(FieldConfig.PRO, canvasW = 400f, canvasH = 2000f)
        assertTrue(tall.toScreenX(-6.0) >= 0f && tall.toScreenX(115.728) <= 400f)
    }

    @Test fun pickFindsBodyUnderCursor() {
        val s = ViewerState()
        val ids = IntArray(64)
        s.world.bodyIds(ids)
        val b = s.world.body(BodyId(ids[0]))
        val x = b.x
        val y = b.y
        assertEquals(BodyId(ids[0]), s.pick(x, y))
        assertNull(s.pick(-5.0, -5.0))
    }

    @Test fun pausedFrameDoesNotTickButStepDoes() {
        val s = ViewerState()
        s.paused = true
        s.frame(0.5)
        assertEquals(0L, s.world.tick)
        s.stepOnce()
        assertEquals(1L, s.world.tick)
    }

    @Test fun speedScalesFrameTime() {
        val s = ViewerState()
        s.speed = 4.0
        s.frame(1.0 / 60.0)
        assertEquals(4L, s.world.tick)
    }

    @Test fun resetRebuildsScenario() {
        val s = ViewerState()
        val h0 = s.world.stateHash()
        repeat(10) { s.stepOnce() }
        s.reset()
        assertEquals(0L, s.world.tick)
        assertEquals(h0, s.world.stateHash())
    }
}
