package io.github.itstrue54.gridironphysics

/**
 * Which bodies a query sees. Mutable so the game can keep one and reuse it. A body passes when its categoryBits
 * share a bit with [maskBits], it is not [ignore]d, and it is not a sensor shape (unless [includeSensors]).
 * Runoff walls are engine geometry, not bodies, and queries never report them.
 */
class QueryFilter(
    var maskBits: Int = Category.ALL,
    var includeSensors: Boolean = false,
    /** [BodyId.value] of a body to skip (usually the asking player), or [NO_BODY]. */
    var ignore: Int = NO_BODY,
) {
    fun ignore(id: BodyId): QueryFilter { ignore = id.value; return this }

    companion object {
        const val NO_BODY = -1
    }
}

/**
 * Result of [World.raycast] and [World.circleCast]. The caller owns it and can reuse it. Valid only when the
 * query returned true.
 */
class RayHit {
    var body: BodyId = BodyId(0)
        internal set

    /** Where along the ray the hit is: 0 at `from`, 1 at `to`. */
    var fraction: Double = 0.0
        internal set

    /** Raycast: the point on the shape's surface. CircleCast: the cast circle's center at the moment of contact. */
    var pointX: Double = 0.0
        internal set
    var pointY: Double = 0.0
        internal set

    /** Unit surface normal at the hit, pointing out of the shape that was hit. */
    var normalX: Double = 0.0
        internal set
    var normalY: Double = 0.0
        internal set

    override fun toString(): String = "RayHit(body=${body.value}, fraction=$fraction, point=($pointX, $pointY), normal=($normalX, $normalY))"
}
