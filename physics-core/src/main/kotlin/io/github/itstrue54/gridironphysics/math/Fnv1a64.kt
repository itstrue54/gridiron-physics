package io.github.itstrue54.gridironphysics.math

/** 64-bit FNV-1a over raw bytes. Longs are mixed as 8 little-endian bytes, doubles by their raw bits. */
object Fnv1a64 {
    const val OFFSET: Long = -0x340d631b7bdddcdbL // 0xcbf29ce484222325
    const val PRIME: Long = 0x100000001b3L

    fun mixByte(h: Long, b: Int): Long = (h xor (b.toLong() and 0xFF)) * PRIME

    fun mixLong(h: Long, v: Long): Long {
        var r = h
        for (i in 0 until 8) r = mixByte(r, (v ushr (8 * i)).toInt())
        return r
    }

    fun mixDouble(h: Long, d: Double): Long = mixLong(h, java.lang.Double.doubleToRawLongBits(d))
}
