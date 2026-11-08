package io.github.itstrue54.gridironphysics.internal

/**
 * The ball's 2.5D state. In M1 it is always Dead at the origin, but it is already part of
 * stateHash() so the hash layout does not change when ball flight arrives in M6.
 */
internal class BallState {
    @JvmField var x = 0.0
    @JvmField var y = 0.0
    @JvmField var z = 0.0
    @JvmField var vx = 0.0
    @JvmField var vy = 0.0
    @JvmField var vz = 0.0

    /** 0 = Dead. Later: Airborne, Loose, Carried. */
    @JvmField var mode = MODE_DEAD

    companion object {
        const val MODE_DEAD = 0
    }
}
