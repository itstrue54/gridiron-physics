package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.Stance

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
    @JvmField var categoryBits = 0
    @JvmField var maskBits = 0
    @JvmField var material = 0

    /** Seconds spent below the sleep speed. */
    @JvmField var sleepTime = 0.0

    /** Largest distance from the body origin to any point of the shape. */
    @JvmField var extent = 0.0

    /** World-space shape, refreshed in the broad phase. */
    @JvmField val geom = Geom()

    /** Pose at the start of the current step; the relax phase measures motion from it. */
    @JvmField var x0 = 0.0
    @JvmField var y0 = 0.0
    @JvmField var angle0 = 0.0

    /** Upper bound on how fast any point of the body moves (m/s), for speculative margins. */
    @JvmField var speedBound = 0.0

    /** Force (N) applied this tick by locomotion; cleared after velocity integration. */
    @JvmField var fx = 0.0
    @JvmField var fy = 0.0

    @JvmField var stance = Stance.Upright

    /** The collider the body was created with: its Upright shape. */
    @JvmField val uprightShape: ShapeDef = shape

    /** Lying collider for Diving and Down, built on the first stance change. */
    @JvmField var lyingShape: ShapeDef.Capsule? = null

    /** The material the body was created with; Upright and Diving use it, Down uses DOWNED_PLAYER. */
    @JvmField var baseMaterial = 0

    /** Velocity after forces, before the contact solver: "before impact" for hit events. */
    @JvmField var preVx = 0.0
    @JvmField var preVy = 0.0

    /** Bit per FieldLine the shape overlapped at the end of the last step. */
    @JvmField var lineMask = 0

    /** True once the shape touched a sideline or end line; false again when it is fully in bounds. */
    @JvmField var outOfBounds = false

    /** Set by World.setAthlete; null for bodies without locomotion. */
    @JvmField var loco: LocomotionController? = null

    val sensor: Boolean get() = shape.sensor

    /** Circle and capsule radius; for polygons and segments, the bounding radius about the origin. */
    val radius: Double
        get() = when (val s = shape) {
            is ShapeDef.Circle -> s.radius
            is ShapeDef.Capsule -> s.radius
            else -> extent
        }

    /** Dynamic and awake, or kinematic and moving: a body whose contacts must be computed. */
    val active: Boolean
        get() = when (type) {
            BodyType.Dynamic -> awake
            BodyType.Kinematic -> vx != 0.0 || vy != 0.0 || angularVelocity != 0.0
            BodyType.Static -> false
        }
}
