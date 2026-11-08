package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.Contact

/**
 * Read-only view of one contact from the last step. World reuses a single instance: a view is valid only
 * until the next call to [World.contact] or [World.step]. A contact exists while two shapes are touching or
 * about to touch (speculative points have positive [separation]). The normal points from A to B.
 */
class ContactView internal constructor() {
    internal var c: Contact? = null
    private val contact: Contact get() = c ?: throw IllegalStateException("ContactView not bound")

    /** The lower-id body. */
    val bodyA: BodyId get() = BodyId(contact.idA)

    /** The higher-id body. Throws when B is a runoff wall; check [wall] first. */
    val bodyB: BodyId
        get() {
            check(contact.wall == NO_WALL) { "bodyB is runoff wall ${contact.wall}, not a body" }
            return BodyId(contact.idB)
        }

    /** [NO_WALL], or which engine-owned static B is: a runoff wall or a goal-post base ([POST_HOME], [POST_AWAY]). */
    val wall: Int get() = contact.wall

    /** Sensor contacts report overlap and never push (sensor shapes, ball against player). */
    val sensor: Boolean get() = contact.sensor

    /** True when the shapes actually overlap, not just within the speculative margin. */
    val touching: Boolean
        get() {
            val m = contact.m
            for (i in 0 until m.count) if (m.sep[i] < 0.0) return true
            return false
        }

    val pointCount: Int get() = contact.m.count
    val normalX: Double get() = contact.m.nx
    val normalY: Double get() = contact.m.ny
    fun pointX(i: Int): Double = contact.m.px[check(i)]
    fun pointY(i: Int): Double = contact.m.py[check(i)]

    /** Signed distance between the surfaces at point [i]: negative means overlap. */
    fun separation(i: Int): Double = contact.m.sep[check(i)]

    /** Accumulated normal impulse (N·s) applied at point [i] this tick. */
    fun normalImpulse(i: Int): Double = contact.normalImpulse[check(i)]
    fun tangentImpulse(i: Int): Double = contact.tangentImpulse[check(i)]

    private fun check(i: Int): Int {
        require(i in 0 until contact.m.count) { "point $i out of range 0 until ${contact.m.count}" }
        return i
    }

    companion object {
        const val NO_WALL = -1

        /** Wall along y = −runoff. */
        const val WALL_NEAR = 0

        /** Wall along y = width + runoff. */
        const val WALL_FAR = 1

        /** Wall along x = −runoff. */
        const val WALL_HOME = 2

        /** Wall along x = length + runoff. */
        const val WALL_AWAY = 3

        /** The padded goal-post base behind the home end line (x < 0). Present when [WorldConfig.goalPosts] is on. */
        const val POST_HOME = 4

        /** The padded goal-post base behind the away end line. */
        const val POST_AWAY = 5
    }
}
