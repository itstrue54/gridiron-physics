package io.github.itstrue54.gridironphysics.math

/** Mutable 2D value holder. Callers own and reuse instances; the engine never returns a fresh one during play. */
class Vec2(var x: Double = 0.0, var y: Double = 0.0) {
    fun set(x: Double, y: Double): Vec2 { this.x = x; this.y = y; return this }
    fun set(o: Vec2): Vec2 { x = o.x; y = o.y; return this }
    fun lengthSq(): Double = x * x + y * y
    fun length(): Double = DMath.sqrt(lengthSq())
    override fun toString(): String = "Vec2($x, $y)"
}
