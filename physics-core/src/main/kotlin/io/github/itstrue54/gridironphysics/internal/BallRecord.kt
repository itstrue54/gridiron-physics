package io.github.itstrue54.gridironphysics.internal

/** The ball's 2.5D state (World owns one). [mode] is a [io.github.itstrue54.gridironphysics.BallState] ordinal. */
internal class BallRecord {
    @JvmField var x = 0.0
    @JvmField var y = 0.0
    @JvmField var z = 0.0
    @JvmField var vx = 0.0
    @JvmField var vy = 0.0
    @JvmField var vz = 0.0
    @JvmField var prevX = 0.0
    @JvmField var prevY = 0.0
    @JvmField var prevZ = 0.0

    @JvmField var mode = MODE_DEAD

    /** [io.github.itstrue54.gridironphysics.FlightMode] ordinal. */
    @JvmField var flightMode = 0

    companion object {
        const val MODE_DEAD = 0
        const val MODE_AIRBORNE = 1
        const val MODE_LOOSE = 2
        const val MODE_CARRIED = 3
    }
}
