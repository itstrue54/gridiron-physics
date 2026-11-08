package io.github.itstrue54.gridironphysics.internal

/** One pair's contact for the current tick, with solver state. Pooled and reused; never allocated in step(). */
internal class Contact {
    @JvmField var a: Body? = null
    @JvmField var b: Body? = null

    /** Sort and warm-start key: (key of A) shl 32 or (key of B). Walls sort after every body. */
    @JvmField var key = 0L
    @JvmField var idA = 0
    @JvmField var idB = 0

    /** Runoff wall index of B, or -1. */
    @JvmField var wall = -1
    @JvmField var sensor = false
    @JvmField var friction = 0.0
    @JvmField var restitution = 0.0

    /** False when neither side can move (nothing to solve). */
    @JvmField var solve = false
    @JvmField val m = Manifold()

    @JvmField val rAx = DoubleArray(2)
    @JvmField val rAy = DoubleArray(2)
    @JvmField val rBx = DoubleArray(2)
    @JvmField val rBy = DoubleArray(2)
    @JvmField val normalMass = DoubleArray(2)
    @JvmField val tangentMass = DoubleArray(2)
    @JvmField val normalImpulse = DoubleArray(2)
    @JvmField val tangentImpulse = DoubleArray(2)
    @JvmField val maxNormalImpulse = DoubleArray(2)

    /** Relative normal velocity before solving, for restitution. */
    @JvmField val relativeVelocity = DoubleArray(2)
}
