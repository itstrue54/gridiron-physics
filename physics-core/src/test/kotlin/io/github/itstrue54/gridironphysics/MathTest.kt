package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Fnv1a64
import io.github.itstrue54.gridironphysics.math.SplitMix64
import io.github.itstrue54.gridironphysics.math.Vec2
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class MathTest {
    @Test fun splitMix64MatchesReferenceVectors() {
        val r = SplitMix64(0L)
        assertEquals(0xe220a8397b1dcdafuL.toLong(), r.nextLong())
        assertEquals(0x6e789e6aa1b965f4uL.toLong(), r.nextLong())
        assertEquals(0x06c45d188009454fuL.toLong(), r.nextLong())
    }

    @Test fun splitMix64DoubleInUnitRange() {
        val r = SplitMix64(42L)
        repeat(10_000) { val d = r.nextDouble(); assertTrue(d >= 0.0 && d < 1.0) }
        repeat(1_000) { val d = r.nextDouble(-2.0, 3.0); assertTrue(d >= -2.0 && d < 3.0) }
    }

    @Test fun fnvMatchesReferenceVectors() {
        assertEquals(0xcbf29ce484222325uL.toLong(), Fnv1a64.OFFSET)
        assertEquals(0xaf63dc4c8601ec8cuL.toLong(), Fnv1a64.mixByte(Fnv1a64.OFFSET, 'a'.code))
    }

    @Test fun fnvMixLongIsLittleEndianBytes() {
        val v = 0x0102030405060708L
        var h = Fnv1a64.OFFSET
        for (i in 0 until 8) h = Fnv1a64.mixByte(h, ((v ushr (8 * i)) and 0xFF).toInt())
        assertEquals(h, Fnv1a64.mixLong(Fnv1a64.OFFSET, v))
        assertEquals(
            Fnv1a64.mixLong(Fnv1a64.OFFSET, java.lang.Double.doubleToRawLongBits(1.5)),
            Fnv1a64.mixDouble(Fnv1a64.OFFSET, 1.5),
        )
    }

    @Test fun yardsRoundTrip() {
        assertEquals(109.728, Yards.toMeters(120.0), 1e-12)
        assertEquals(48.768, Yards.toMeters(160.0 / 3.0), 1e-12)
        assertEquals(45.0, Yards.fromMeters(Yards.toMeters(45.0)), 1e-12)
    }

    /**
     * sin/cos reduce the argument to |r| ≤ π/4 themselves, then call StrictMath on r: StrictMath's own reduction
     * allocates on JDK 17+ desktop runtimes. Pure IEEE arithmetic plus StrictMath keeps the results bit-identical on
     * every platform; they may differ from StrictMath.sin(x) itself by at most 1 ulp.
     */
    @Test fun wrapPiMapsAnglesIntoMinusPiExclusivePiInclusive() {
        assertEquals(DMath.PI, DMath.wrapPi(-DMath.PI))
        assertEquals(DMath.PI, DMath.wrapPi(DMath.PI))
        assertEquals(0.5, DMath.wrapPi(0.5))
        assertEquals(DMath.PI / 2, DMath.wrapPi(-1.5 * DMath.PI), 1e-12)
        assertEquals(-DMath.PI / 2, DMath.wrapPi(3.5 * DMath.PI), 1e-12)
    }

    @Test fun dMathSinCosMatchStrictMathWithin1UlpAndNeverAllocate() {
        val rng = SplitMix64(9L)
        val xs = DoubleArray(20000) { i ->
            when (i % 4) {
                0 -> rng.nextDouble(-4.0, 4.0)
                1 -> rng.nextDouble(-1000.0, 1000.0)
                2 -> (i / 4 - 2500) * (DMath.PI / 4) // multiples of π/4: quadrant edges
                else -> rng.nextDouble(-1e-3, 1e-3)
            }
        }
        for (x in xs) {
            assertTrue(kotlin.math.abs(DMath.sin(x) - StrictMath.sin(x)) <= Math.ulp(StrictMath.sin(x)), "sin($x)")
            assertTrue(kotlin.math.abs(DMath.cos(x) - StrictMath.cos(x)) <= Math.ulp(StrictMath.cos(x)), "cos($x)")
        }
        assertEquals(0.0, DMath.sin(0.0)); assertEquals(1.0, DMath.cos(0.0))
        assertEquals(-1.0, DMath.cos(DMath.PI)); assertEquals(1.0, DMath.sin(DMath.PI / 2))
        assertTrue(DMath.sin(Double.NaN).isNaN() && DMath.cos(Double.POSITIVE_INFINITY).isNaN())
        assertEquals(StrictMath.sin(1e9), DMath.sin(1e9)) // huge arguments fall back to StrictMath

        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        var sink = 0.0
        repeat(5) { for (x in xs) sink += all(x) } // warm up: the JIT must settle before measuring
        val tid = Thread.currentThread().id
        val before = mx.getThreadAllocatedBytes(tid)
        for (x in xs) sink += all(x)
        assertEquals(0L, mx.getThreadAllocatedBytes(tid) - before, "bytes allocated by 80000 DMath calls")
        assertTrue(sink.isFinite())
    }

    private fun all(x: Double) = DMath.sin(x) + DMath.cos(x) + DMath.atan2(x, 1.3) + DMath.exp(-DMath.abs(x) / 1000.0)

    @Test fun dMathUsesStrictMath() {
        assertEquals(StrictMath.atan2(0.3, -0.7), DMath.atan2(0.3, -0.7))
        assertEquals(StrictMath.exp(-2.5), DMath.exp(-2.5))
        assertEquals(StrictMath.log(7.0), DMath.log(7.0))
        assertEquals(5.0, DMath.hypot2(3.0, 4.0))
        assertEquals(2.0, DMath.clamp(5.0, -1.0, 2.0))
    }

    @Test fun vec2SetAndLength() {
        val v = Vec2().set(3.0, 4.0)
        assertEquals(5.0, v.length())
        assertEquals(25.0, v.lengthSq())
        assertEquals(3.0, Vec2().set(v).x)
    }
}
