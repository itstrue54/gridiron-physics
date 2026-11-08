package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.Body

/**
 * Read-only view of one body. World reuses a single instance: a view is valid only until the next
 * call to [World.body]. Copy out what you need.
 */
class BodyView internal constructor() {
    internal var b: Body? = null
    private val body: Body get() = b ?: throw IllegalStateException("BodyView not bound")

    val id: BodyId get() = BodyId(body.id)
    val type: BodyType get() = body.type
    val x: Double get() = body.x
    val y: Double get() = body.y

    /** Position before the most recent step, for interpolated drawing. */
    val prevX: Double get() = body.prevX
    val prevY: Double get() = body.prevY
    val angle: Double get() = body.angle
    val vx: Double get() = body.vx
    val vy: Double get() = body.vy
    val angularVelocity: Double get() = body.angularVelocity
    val mass: Double get() = body.mass
    val invMass: Double get() = body.invMass
    val inertia: Double get() = body.inertia
    val linearDamping: Double get() = body.linearDamping
    val angularDamping: Double get() = body.angularDamping
    /** Circle and capsule radius; for polygons and segments, the bounding radius about the body origin. */
    val radius: Double get() = body.radius
    val shape: ShapeDef get() = body.shape
    val userData: Int get() = body.userData
    val bullet: Boolean get() = body.bullet
    val awake: Boolean get() = body.awake
    val categoryBits: Int get() = body.categoryBits
    val maskBits: Int get() = body.maskBits
    val material: Int get() = body.material
}
