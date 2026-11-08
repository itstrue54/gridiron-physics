package io.github.itstrue54.gridironphysics.math

/** Mutable 3D value holder (x down field, y across, z height). Used by ball flight. */
class Vec3(var x: Double = 0.0, var y: Double = 0.0, var z: Double = 0.0) {
    fun set(x: Double, y: Double, z: Double): Vec3 { this.x = x; this.y = y; this.z = z; return this }
    fun set(o: Vec3): Vec3 { x = o.x; y = o.y; z = o.z; return this }
    override fun toString(): String = "Vec3($x, $y, $z)"
}
