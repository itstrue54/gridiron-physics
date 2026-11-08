package io.github.itstrue54.gridironphysics

/**
 * Sees every state-changing call the game makes on a [World], stamped with the tick it happened on.
 * Replays (M7) are built from this log, so every new public mutator must add a method here.
 */
interface CommandRecorder {
    fun onCreateBody(tick: Long, id: BodyId, def: BodyDef)
    fun onDestroyBody(tick: Long, id: BodyId)
}
