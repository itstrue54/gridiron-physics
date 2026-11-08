package io.github.itstrue54.gridironphysics

/** Collision shape of a body. M1 has circles only; capsule, polygon and segment arrive in M2. */
sealed class ShapeDef {
    class Circle(val radius: Double) : ShapeDef() {
        override fun toString(): String = "Circle(r=$radius)"
    }
}
