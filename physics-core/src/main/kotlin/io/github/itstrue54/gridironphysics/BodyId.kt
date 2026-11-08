package io.github.itstrue54.gridironphysics

/**
 * Handle to a body: a storage slot (high bits) plus a generation (low 20 bits). Destroying a body
 * frees its slot for a later body, but the new body gets a new generation, so a stale handle is
 * rejected instead of silently pointing at someone else. A slot can be reused about a million times
 * before its generation wraps. Ids sort by slot, which is the order the engine iterates in.
 */
@JvmInline
value class BodyId(val value: Int) {
    internal val slot: Int get() = value ushr GEN_BITS
    internal val generation: Int get() = value and GEN_MASK

    internal companion object {
        const val GEN_BITS = 20
        const val GEN_MASK = (1 shl GEN_BITS) - 1

        /** Slots fit in the remaining 11 bits of a non-negative Int. */
        const val MAX_SLOTS = 1 shl (31 - GEN_BITS)

        fun of(slot: Int, generation: Int): BodyId = BodyId((slot shl GEN_BITS) or (generation and GEN_MASK))
    }
}

enum class BodyType {
    /** Moved by forces and contacts: players, the loose ball. */
    Dynamic,

    /** Moved only by its set velocity, unaffected by contacts or damping: officials, scripted movers. */
    Kinematic,

    /** Never moves and has no velocity: goal posts, runoff walls. */
    Static,
}
