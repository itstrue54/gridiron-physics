package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class AdvanceTest {
    private fun world() = World(WorldConfig(), FieldConfig.PRO, seed = 1L)

    @Test fun oneFrameOneTick() {
        val w = world()
        w.advance(1.0 / 60.0)
        assertEquals(1L, w.tick)
    }

    // Review Focus 3
    @Test fun sixtyFramesSixtyTicks() {
        val w = world()
        repeat(60) { w.advance(1.0 / 60.0) }
        assertEquals(60L, w.tick)
    }

    @Test fun halfTickGivesAlphaHalf() {
        val w = world()
        val a = w.advance(0.5 / 60.0)
        assertEquals(0L, w.tick)
        assertEquals(0.5, a, 1e-9)
        assertEquals(a, w.alpha)
    }

    // Review Focus 1
    @Test fun hugeFrameClampsToFourTicks() {
        val w = world()
        val a = w.advance(10.0)
        assertEquals(4L, w.tick)
        assertTrue(a >= 0.0 && a < 1.0)
        w.advance(0.0)
        assertEquals(4L, w.tick)
    }

    // Review Focus 2
    @Test fun zeroNegativeNaN() {
        val w = world()
        w.advance(0.0)
        w.advance(-1.0)
        assertEquals(0L, w.tick)
        assertFailsWith<IllegalArgumentException> { w.advance(Double.NaN) }
        assertFailsWith<IllegalArgumentException> { w.advance(Double.POSITIVE_INFINITY) }
    }

    @Test fun at144HzTicksAverageSixtyPerSecond() {
        val w = world()
        repeat(144) { w.advance(1.0 / 144.0) }
        assertEquals(60L, w.tick)
    }

    @Test fun at30HzTwoTicksPerFrame() {
        val w = world()
        repeat(30) { w.advance(1.0 / 30.0) }
        assertEquals(60L, w.tick)
    }
}
