package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.ShapeDef

/** Internal mutable body record. One per live body, stored in World's id-indexed array. */
internal class Body(
    @JvmField val id: Int,
    @JvmField val type: BodyType,
    @JvmField var shape: ShapeDef,
) {
    @JvmField var x = 0.0
    @JvmField var y = 0.0
    @JvmField var prevX = 0.0
    @JvmField var prevY = 0.0
    @JvmField var angle = 0.0
    @JvmField var vx = 0.0
    @JvmField var vy = 0.0
    @JvmField var angularVelocity = 0.0
    @JvmField var mass = 0.0
    @JvmField var invMass = 0.0
    @JvmField var inertia = 0.0
    @JvmField var invInertia = 0.0
    @JvmField var linearDamping = 0.0
    @JvmField var angularDamping = 0.0
    @JvmField var userData = 0
    @JvmField var bullet = false
    @JvmField var awake = true

    val radius: Double
        get() = when (val s = shape) {
            is ShapeDef.Circle -> s.radius
        }
}
