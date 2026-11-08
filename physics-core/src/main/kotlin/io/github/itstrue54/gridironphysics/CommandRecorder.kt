package io.github.itstrue54.gridironphysics

/**
 * Sees every state-changing call the game makes on a [World], stamped with the tick it happened on.
 * Replays (M7) are built from this log, so every new public mutator must add a method here.
 */
interface CommandRecorder {
    /**
     * [def] is the caller's object and is only valid during this call: games reuse one BodyDef for many
     * bodies. To keep it, copy it with [BodyDef.copyFrom] into a def the recorder owns.
     */
    fun onCreateBody(tick: Long, id: BodyId, def: BodyDef)
    fun onDestroyBody(tick: Long, id: BodyId)
    fun onSetAthlete(tick: Long, id: BodyId, params: AthleteParams)

    /** [intent] is the caller's object and is only valid during this call; copy it with [LocomotionIntent.copyFrom]. */
    fun onSetIntent(tick: Long, id: BodyId, intent: LocomotionIntent)
    fun onSetStance(tick: Long, id: BodyId, stance: Stance)
    fun onSetSurface(tick: Long, surface: SurfaceConfig)

    // Joint defs are immutable, so a recorder may keep them.
    fun onCreateGrapple(tick: Long, id: JointId, tackler: BodyId, carrier: BodyId, def: GrappleDef)
    fun onCreateBlock(tick: Long, id: JointId, blocker: BodyId, defender: BodyId, def: BlockDef)
    fun onCreateDistance(tick: Long, id: JointId, a: BodyId, b: BodyId, def: DistanceDef)
    fun onCreateWeld(tick: Long, id: JointId, a: BodyId, b: BodyId, def: WeldDef)

    /** Only calls to [World.destroyJoint]; joints that break, or go with a destroyed body, are not commands. */
    fun onDestroyJoint(tick: Long, id: JointId)
    fun onSetBallCarrier(tick: Long, id: BodyId?)
    fun onSetAttackDirection(tick: Long, direction: Int)
}
