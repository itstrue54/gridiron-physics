package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.FieldConfig

/**
 * The two goal-post assemblies in 3D, from [FieldConfig]: at each end line two uprights (vertical, from the crossbar
 * up by [FieldConfig.uprightHeightAboveCrossbarM]), the crossbar between them, and a gooseneck: an arm from the middle
 * of the crossbar back to a pole [FieldConfig.goalPostOffsetM] behind the end line, standing on the ground. Each piece
 * is a capsule, grown by the ball's radius so the ball is swept as a point. Engine-owned, immutable after construction.
 */
internal class GoalPosts(f: FieldConfig, ballRadius: Double) {
    val count = 10
    @JvmField val ax = DoubleArray(count)
    @JvmField val ay = DoubleArray(count)
    @JvmField val az = DoubleArray(count)
    @JvmField val bx = DoubleArray(count)
    @JvmField val by = DoubleArray(count)
    @JvmField val bz = DoubleArray(count)
    @JvmField val r = DoubleArray(count)

    /** Nothing of the assembly is farther than this from an end line along x (m). */
    @JvmField val reachX: Double

    private val sweep = Sweep3()

    /** Output of [sweep]. */
    @JvmField var t = 0.0
    @JvmField var nx = 0.0
    @JvmField var ny = 0.0
    @JvmField var nz = 0.0

    private val length = f.lengthM

    init {
        val cy = f.widthM / 2
        val half = f.uprightSpacingM / 2
        val bar = f.crossbarHeightM
        val top = bar + f.uprightHeightAboveCrossbarM
        val post = f.uprightRadiusM + ballRadius
        val neck = GOOSENECK_RADIUS + ballRadius
        var i = 0
        fun piece(x1: Double, y1: Double, z1: Double, x2: Double, y2: Double, z2: Double, rr: Double) {
            ax[i] = x1; ay[i] = y1; az[i] = z1; bx[i] = x2; by[i] = y2; bz[i] = z2; r[i] = rr; i++
        }
        for (end in 0..1) {
            val x = if (end == 0) 0.0 else f.lengthM
            val back = if (end == 0) -f.goalPostOffsetM else f.goalPostOffsetM
            piece(x, cy - half, bar, x, cy - half, top, post)
            piece(x, cy + half, bar, x, cy + half, top, post)
            piece(x, cy - half, bar, x, cy + half, bar, post)
            piece(x, cy, bar, x + back, cy, bar, neck)
            piece(x + back, cy, 0.0, x + back, cy, bar, neck)
        }
        reachX = f.goalPostOffsetM + neck + post
    }

    /**
     * Sweeps the ball's center from (px, py, pz) by (dx, dy, dz) and returns the index of the first piece it touches,
     * filling [t] and the normal, or −1.
     */
    fun sweep(px: Double, py: Double, pz: Double, dx: Double, dy: Double, dz: Double): Int {
        val x0 = if (dx < 0.0) px + dx else px
        val x1 = if (dx < 0.0) px else px + dx
        // Far from both end lines: nothing to hit.
        if ((x1 < -reachX || x0 > reachX) && (x1 < length - reachX || x0 > length + reachX)) return -1
        var best = -1
        var bt = 2.0
        for (k in 0 until count) {
            if (!sweep.capsule(px, py, pz, dx, dy, dz, ax[k], ay[k], az[k], bx[k], by[k], bz[k], r[k]) || sweep.t >= bt) continue
            best = k; bt = sweep.t
            nx = sweep.nx; ny = sweep.ny; nz = sweep.nz
        }
        t = bt
        return best
    }

    companion object {
        /** The gooseneck's arm and pole (m). */
        const val GOOSENECK_RADIUS = 0.1
    }
}
