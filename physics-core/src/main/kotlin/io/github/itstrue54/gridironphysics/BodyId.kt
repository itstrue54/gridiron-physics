package io.github.itstrue54.gridironphysics

/** Handle to a body. Ids are issued in ascending order and never reused within a World. */
@JvmInline
value class BodyId(val value: Int)

enum class BodyType {
    /** Moved by forces and contacts: players, the loose ball. */
    Dynamic,

    /** Moved only by its set velocity, unaffected by contacts or damping: officials, scripted movers. */
    Kinematic,

    /** Never moves: goal posts, runoff walls. */
    Static,
}
