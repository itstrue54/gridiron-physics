package io.github.itstrue54.gridironphysics.math

/**
 * The only place physics-core does transcendental math. Every function is either [StrictMath] or plain IEEE
 * arithmetic, so results are bit-identical on every JVM and on ART. A future multiplatform build swaps this
 * object out.
 */
object DMath {
    const val PI: Double = 3.141592653589793

    /**
     * sin and cos are computed here rather than by StrictMath, because StrictMath.sin/cos allocate a scratch
     * array on every call on JDK 17+ desktop runtimes (the Java port of fdlibm), and step() must not allocate.
     * They use fdlibm's method: Cody–Waite reduction to r ∈ [−π/4, π/4] kept as a head + tail pair, then the
     * minimax polynomial kernels. Pure IEEE arithmetic, so bit-identical everywhere; within 1 ulp of StrictMath.
     * Huge |a| (≥ 1e6, never seen in play) and NaN/∞ fall back to StrictMath.
     */
    fun sin(a: Double): Double {
        if (abs(a) <= PI_4) return kernelSin(a, 0.0, false)
        if (!(abs(a) < REDUCE_LIMIT)) return StrictMath.sin(a)
        return reduced(a) { q, y0, y1 ->
            when (q) {
                0 -> kernelSin(y0, y1, true)
                1 -> kernelCos(y0, y1)
                2 -> -kernelSin(y0, y1, true)
                else -> -kernelCos(y0, y1)
            }
        }
    }

    fun cos(a: Double): Double {
        if (abs(a) <= PI_4) return kernelCos(a, 0.0)
        if (!(abs(a) < REDUCE_LIMIT)) return StrictMath.cos(a)
        return reduced(a) { q, y0, y1 ->
            when (q) {
                0 -> kernelCos(y0, y1)
                1 -> -kernelSin(y0, y1, true)
                2 -> -kernelCos(y0, y1)
                else -> kernelSin(y0, y1, true)
            }
        }
    }

    fun atan2(y: Double, x: Double): Double = StrictMath.atan2(y, x)
    fun exp(a: Double): Double = StrictMath.exp(a)
    fun log(a: Double): Double = StrictMath.log(a)

    /** IEEE sqrt is correctly rounded everywhere. */
    fun sqrt(a: Double): Double = StrictMath.sqrt(a)

    fun abs(a: Double): Double = if (a < 0.0) -a else if (a == 0.0) 0.0 else a
    fun min(a: Double, b: Double): Double = StrictMath.min(a, b)
    fun max(a: Double, b: Double): Double = StrictMath.max(a, b)

    /** Exact on every platform (no rounding involved). */
    fun floor(a: Double): Double = StrictMath.floor(a)
    /** Wraps an angle to (−π, π]. */
    fun wrapPi(a: Double): Double {
        var r = a - 2.0 * PI * floor((a + PI) / (2.0 * PI))
        if (r <= -PI) r += 2.0 * PI
        return r
    }

    fun clamp(v: Double, lo: Double, hi: Double): Double = if (v < lo) lo else if (v > hi) hi else v

    /** sqrt(x² + y²). Deliberately not StrictMath.hypot: that is slower and we never overflow. */
    fun hypot2(x: Double, y: Double): Double = StrictMath.sqrt(x * x + y * y)

    // ------------------------------------------------------------------ sin/cos internals

    /**
     * Calls [k] with the quadrant q = n mod 4 and a − n·π/2 = y0 + y1, where n = round(a·2/π). π/2 is split into
     * 33-bit parts so every n·part is exact for |n| < 2^20, and the rounding error of each subtraction is kept
     * (TwoSum), so arguments next to a multiple of π/2 keep their precision. Inline: no state, no allocation.
     */
    private inline fun reduced(a: Double, k: (q: Int, y0: Double, y1: Double) -> Double): Double {
        val n = StrictMath.rint(a * TWO_OVER_PI)
        val t = a - n * PIO2_1 // exact
        val w = n * PIO2_2 // exact
        val r1 = t - w
        val bb = r1 - t
        val err = (t - (r1 - bb)) + (-w - bb) // t − w = r1 + err exactly
        val c = (err - n * PIO2_3) - n * PIO2_3T
        val head = r1 + c
        return k((n.toLong() and 3L).toInt(), head, (r1 - head) + c)
    }

    /** sin(x + y) for |x| ≤ ~π/4, |y| ≪ ulp(x); [hasTail] false means y is 0. Minimax polynomial (fdlibm k_sin). */
    private fun kernelSin(x: Double, y: Double, hasTail: Boolean): Double {
        if (abs(x) < TINY) return x
        val z = x * x
        val v = z * x
        val r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)))
        return if (!hasTail) x + v * (S1 + z * r) else x - ((z * (0.5 * y - v * r) - y) - v * S1)
    }

    /** cos(x + y) for |x| ≤ ~π/4, |y| ≪ ulp(x). Minimax polynomial (fdlibm k_cos). */
    private fun kernelCos(x: Double, y: Double): Double {
        val ix = (x.toRawBits() ushr 32).toInt() and 0x7fffffff
        if (ix < 0x3e400000) return 1.0 // |x| < 2^-27
        val z = x * x
        val r = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))))
        if (ix < 0x3FD33333) return 1.0 - (0.5 * z - (z * r - x * y)) // |x| < 0.3
        // Split 1 − z/2 so the subtraction stays exact: qx ≈ x²/4 rounded down to a short double.
        val qx = if (ix > 0x3fe90000) 0.28125 else Double.fromBits((ix - 0x00200000).toLong() shl 32)
        val hz = 0.5 * z - qx
        return (1.0 - qx) - (hz - (z * r - x * y))
    }

    private const val PI_4 = 0.7853981633974483
    private const val TINY = 7.450580596923828e-9 // 2^-27
    private const val TWO_OVER_PI = 0.6366197723675814
    private const val PIO2_1 = 1.57079632673412561417e+00 // first 33 bits of π/2
    private const val PIO2_2 = 6.07710050630396597660e-11 // next 33 bits
    private const val PIO2_3 = 2.02226624871116645580e-21 // next 33 bits
    private const val PIO2_3T = 8.47842766036889956997e-32 // π/2 − (PIO2_1 + PIO2_2 + PIO2_3)

    /** Above this, n·PIO2_k is no longer exact. */
    private const val REDUCE_LIMIT = 1.0e6

    // Minimax coefficients for sin and cos on [−π/4, π/4] (as published with fdlibm, Sun Microsystems).
    private const val S1 = -1.66666666666666324348e-01
    private const val S2 = 8.33333333332248946124e-03
    private const val S3 = -1.98412698298579493134e-04
    private const val S4 = 2.75573137070700676789e-06
    private const val S5 = -2.50507602534068634195e-08
    private const val S6 = 1.58969099521155010221e-10
    private const val C1 = 4.16666666666666019037e-02
    private const val C2 = -1.38888888888741095749e-03
    private const val C3 = 2.48015872894767294178e-05
    private const val C4 = -2.75573143513906633035e-07
    private const val C5 = 2.08757232129817482790e-09
    private const val C6 = -1.13596475577881948265e-11
}
