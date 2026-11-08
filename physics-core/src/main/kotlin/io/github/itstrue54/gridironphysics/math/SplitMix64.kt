package io.github.itstrue54.gridironphysics.math

/** SplitMix64 (Steele, Lea, Flood 2014). The engine's only source of randomness; owned by the World. */
class SplitMix64(seed: Long) {
    var state: Long = seed

    fun nextLong(): Long {
        state += -0x61c8864680b583ebL // 0x9e3779b97f4a7c15
        var z = state
        z = (z xor (z ushr 30)) * -0x40a7b892e31b1a47L // 0xbf58476d1ce4e5b9
        z = (z xor (z ushr 27)) * -0x6b2fb644ecceee15L // 0x94d049bb133111eb
        return z xor (z ushr 31)
    }

    /** Uniform in [0, 1) with 53 bits of precision. */
    fun nextDouble(): Double = (nextLong() ushr 11) * DOUBLE_UNIT

    /** Uniform in [lo, hi). */
    fun nextDouble(lo: Double, hi: Double): Double {
        val d = lo + (hi - lo) * nextDouble()
        return if (d < hi) d else lo // guard the rare rounding up to hi
    }

    private companion object {
        const val DOUBLE_UNIT = 1.0 / (1L shl 53)
    }
}
