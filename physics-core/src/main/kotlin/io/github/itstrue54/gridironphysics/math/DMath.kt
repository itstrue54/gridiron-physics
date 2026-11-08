package io.github.itstrue54.gridironphysics.math

/**
 * The only place physics-core does transcendental math. Backed by [StrictMath] so results are
 * bit-identical on every JVM and on ART. A future multiplatform build swaps this object out.
 */
object DMath {
    const val PI: Double = 3.141592653589793

    fun sin(a: Double): Double = StrictMath.sin(a)
    fun cos(a: Double): Double = StrictMath.cos(a)
    fun atan2(y: Double, x: Double): Double = StrictMath.atan2(y, x)
    fun exp(a: Double): Double = StrictMath.exp(a)
    fun log(a: Double): Double = StrictMath.log(a)

    /** IEEE sqrt is correctly rounded everywhere. */
    fun sqrt(a: Double): Double = StrictMath.sqrt(a)

    fun abs(a: Double): Double = if (a < 0.0) -a else if (a == 0.0) 0.0 else a
    fun min(a: Double, b: Double): Double = StrictMath.min(a, b)
    fun max(a: Double, b: Double): Double = StrictMath.max(a, b)
    fun clamp(v: Double, lo: Double, hi: Double): Double = if (v < lo) lo else if (v > hi) hi else v

    /** sqrt(x² + y²). Deliberately not StrictMath.hypot: that is slower and we never overflow. */
    fun hypot2(x: Double, y: Double): Double = StrictMath.sqrt(x * x + y * y)
}
