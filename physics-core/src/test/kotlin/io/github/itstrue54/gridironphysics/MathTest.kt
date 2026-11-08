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

    @Test fun dMathUsesStrictMath() {
        assertEquals(StrictMath.sin(1.234), DMath.sin(1.234))
        assertEquals(StrictMath.cos(1.234), DMath.cos(1.234))
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
