package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.Joint

/**
 * Handle to a joint: a storage slot plus a generation, like [BodyId]. A joint that breaks or is destroyed
 * frees its slot; a later joint in that slot gets a new generation, so a stale handle is rejected. Ids sort by
 * slot, which is the order the solver runs joints in.
 */
@JvmInline
value class JointId(val value: Int) {
    internal val slot: Int get() = value ushr BodyId.GEN_BITS
    internal val generation: Int get() = value and BodyId.GEN_MASK

    internal companion object {
        fun of(slot: Int, generation: Int): JointId = JointId((slot shl BodyId.GEN_BITS) or (generation and BodyId.GEN_MASK))
    }
}

enum class JointType { Grapple, Block, Distance, Weld }

private fun requireSoft(hertz: Double, dampingRatio: Double, allowRigid: Boolean) {
    require(hertz.isFinite() && (if (allowRigid) hertz >= 0.0 else hertz > 0.0)) {
        "hertz must be finite and ${if (allowRigid) ">= 0" else "> 0"}: $hertz"
    }
    require(dampingRatio.isFinite() && dampingRatio >= 0.0) { "dampingRatio must be finite and >= 0: $dampingRatio" }
}

private fun requireLength(length: Double?) {
    require(length == null || (length.isFinite() && length > 0.0)) { "length must be null or finite and > 0: $length" }
}

/**
 * A wrap-up ([World.createGrapple]): a soft, pull-only rope between the tackler's and the carrier's centers,
 * plus drag on the carrier. Immutable.
 */
data class GrappleDef(
    /**
     * Largest pull (N) the grip holds. The joint never pulls harder; on the first tick the solver needs this much
     * it breaks and emits [EventType.GrappleBroken]. The game sets it from the tackler's strength against the
     * carrier's balance. [Double.POSITIVE_INFINITY] never breaks.
     */
    val breakForce: Double,
    /**
     * Force (N) that brakes the carrier against the ground, up to stopping him (never pushing him backward). Every
     * grapple on a carrier adds its own, so gang tackles stack.
     */
    val dragForce: Double = 0.0,
    /** Rope length (m) between centers; null = their distance when the joint is created. */
    val length: Double? = null,
    /** Spring stiffness as a frequency (Hz, > 0): a grapple is always soft. */
    val hertz: Double = 4.0,
    val dampingRatio: Double = 1.0,
) {
    init {
        require(!breakForce.isNaN() && breakForce > 0.0) { "breakForce must be > 0: $breakForce" }
        require(dragForce.isFinite() && dragForce >= 0.0) { "dragForce must be finite and >= 0: $dragForce" }
        requireLength(length)
        requireSoft(hertz, dampingRatio, allowRigid = false)
    }
}

/**
 * An engaged block ([World.createBlock]). Each side drives toward the other with up to its push force, so the
 * net force moves the pair and the stronger side wins. A push is scaled by leverage, the cosine of the angle
 * between the pusher's heading and the direction to his opponent (0 when he faces away), and tapers to 0 as the
 * pusher's own speed toward his opponent reaches [driveSpeed]. A soft spring keeps the pair at its length, and a
 * lateral constraint keeps them squared up; when holding them squared up takes more than [shedThreshold] the
 * block is shed: the joint breaks and emits [EventType.BlockShed]. Immutable.
 */
data class BlockDef(
    /** The blocker's largest push (N). */
    val blockerPushForce: Double,
    /** The defender's largest push (N). */
    val defenderPushForce: Double,
    /** Lateral force (N) across the joint line that sheds the block. [Double.POSITIVE_INFINITY] never sheds. */
    val shedThreshold: Double,
    /** Speed (m/s) toward the opponent at which a push has faded to 0. */
    val driveSpeed: Double = 2.5,
    /** Engaged distance (m) between centers; null = their distance when the joint is created. */
    val length: Double? = null,
    val hertz: Double = 5.0,
    val dampingRatio: Double = 1.0,
) {
    init {
        require(blockerPushForce.isFinite() && blockerPushForce >= 0.0) { "blockerPushForce must be finite and >= 0: $blockerPushForce" }
        require(defenderPushForce.isFinite() && defenderPushForce >= 0.0) { "defenderPushForce must be finite and >= 0: $defenderPushForce" }
        require(!shedThreshold.isNaN() && shedThreshold > 0.0) { "shedThreshold must be > 0: $shedThreshold" }
        require(driveSpeed.isFinite() && driveSpeed > 0.0) { "driveSpeed must be finite and > 0: $driveSpeed" }
        requireLength(length)
        requireSoft(hertz, dampingRatio, allowRigid = false)
    }
}

/** A two-sided distance joint between body centers ([World.createDistance]). Immutable. */
data class DistanceDef(
    /** Length (m); null = the distance when the joint is created. */
    val length: Double? = null,
    /** 0 = rigid (stiff, solved like a contact); > 0 = a spring with this frequency (Hz). */
    val hertz: Double = 0.0,
    val dampingRatio: Double = 1.0,
) {
    init {
        requireLength(length)
        requireSoft(hertz, dampingRatio, allowRigid = true)
    }
}

/** Glues two bodies in their relative position and angle at creation ([World.createWeld]). Immutable. */
data class WeldDef(
    /** 0 = rigid; > 0 = soft with this frequency (Hz). */
    val hertz: Double = 0.0,
    val dampingRatio: Double = 1.0,
) {
    init {
        requireSoft(hertz, dampingRatio, allowRigid = true)
    }
}

/**
 * Read-only view of one joint, as of the end of the last step. World reuses a single instance: a view is valid
 * only until the next call to [World.joint].
 */
class JointView internal constructor() {
    internal var j: Joint? = null
    private val joint: Joint get() = j ?: throw IllegalStateException("JointView not bound")

    val id: JointId get() = JointId(joint.id)
    val type: JointType get() = joint.type

    /** Tackler, blocker, or the first body passed to createDistance / createWeld. */
    val bodyA: BodyId get() = BodyId(joint.a.id)

    /** Carrier, defender, or the second body. */
    val bodyB: BodyId get() = BodyId(joint.b.id)

    /** Length (m) the joint holds the centers at (weld: the distance between the centers at creation). */
    val restLength: Double get() = joint.restLength

    /** Force (N) along the joint last step: the grapple's pull, the block's or distance joint's push or pull, the weld's linear force. */
    val force: Double get() = joint.force()

    /** Block: force (N) across the joint line that kept the pair squared up last step. 0 for other joints. */
    val lateralForce: Double get() = joint.lateralForce()

    /** Grapple: [GrappleDef.breakForce]; block: [BlockDef.shedThreshold]; distance and weld: infinity. */
    val limit: Double get() = joint.limit()

    /** How close the joint is to breaking: [force] / limit for a grapple, [lateralForce] / limit for a block, else 0. */
    val load: Double get() = joint.load()

    /** Block: the blocker's push (N) last step, after leverage and taper. */
    val blockerPush: Double get() = joint.pushA

    /** Block: the defender's push (N) last step, after leverage and taper. */
    val defenderPush: Double get() = joint.pushB

    /** Where the joint attaches (body centers), for drawing. */
    val anchorAX: Double get() = joint.a.x
    val anchorAY: Double get() = joint.a.y
    val anchorBX: Double get() = joint.b.x
    val anchorBY: Double get() = joint.b.y
}
