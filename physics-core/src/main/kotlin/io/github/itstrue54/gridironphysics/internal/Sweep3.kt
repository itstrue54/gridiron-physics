package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.math.DMath

/**
 * A point moving p(t) = p + t·d, t ∈ [0, 1], against a 3D capsule (segment a–b grown by r). Sweeping a sphere of
 * radius rb against a post of radius rp is this with r = rb + rp. Results go in the fields; allocation-free.
 */
internal class Sweep3 {
    /** Time of impact in [0, 1]. */
    @JvmField var t = 0.0

    /** Outward unit normal of the capsule at the impact (from the axis toward the point). */
    @JvmField var nx = 0.0
    @JvmField var ny = 0.0
    @JvmField var nz = 0.0

    /**
     * Returns true and fills the fields when the point enters the capsule during the sweep. A point that starts
     * inside counts as a hit at t = 0 only while it moves deeper (so a ball pushed out and leaving is let go).
     */
    fun capsule(
        px: Double, py: Double, pz: Double, dx: Double, dy: Double, dz: Double,
        ax: Double, ay: Double, az: Double, bx: Double, by: Double, bz: Double, r: Double,
    ): Boolean {
        val ex = bx - ax; val ey = by - ay; val ez = bz - az
        val len2 = ex * ex + ey * ey + ez * ez
        // Start inside?
        val s0 = if (len2 > 0.0) DMath.clamp(((px - ax) * ex + (py - ay) * ey + (pz - az) * ez) / len2, 0.0, 1.0) else 0.0
        val cx = px - (ax + s0 * ex); val cy = py - (ay + s0 * ey); val cz = pz - (az + s0 * ez)
        val c2 = cx * cx + cy * cy + cz * cz
        if (c2 < r * r) {
            val c = DMath.sqrt(c2)
            if (c > 0.0) { nx = cx / c; ny = cy / c; nz = cz / c } else {
                val dl = DMath.sqrt(dx * dx + dy * dy + dz * dz)
                if (dl == 0.0) return false
                nx = -dx / dl; ny = -dy / dl; nz = -dz / dl
            }
            if (dx * nx + dy * ny + dz * nz >= 0.0) return false
            t = 0.0
            return true
        }
        var best = 2.0
        // The cylinder side: solve |m⊥ + t·d⊥| = r in the plane across the axis, then check the hit lies along the segment.
        if (len2 > 0.0) {
            val len = DMath.sqrt(len2)
            val ux = ex / len; val uy = ey / len; val uz = ez / len
            val mx = px - ax; val my = py - ay; val mz = pz - az
            val md = mx * ux + my * uy + mz * uz
            val dd = dx * ux + dy * uy + dz * uz
            val mpx = mx - md * ux; val mpy = my - md * uy; val mpz = mz - md * uz
            val dpx = dx - dd * ux; val dpy = dy - dd * uy; val dpz = dz - dd * uz
            val a = dpx * dpx + dpy * dpy + dpz * dpz
            if (a > 1e-24) {
                val b = mpx * dpx + mpy * dpy + mpz * dpz
                val c = mpx * mpx + mpy * mpy + mpz * mpz - r * r
                val disc = b * b - a * c
                if (b < 0.0 && disc >= 0.0) {
                    val tt = (-b - DMath.sqrt(disc)) / a
                    val s = md + tt * dd
                    if (tt in 0.0..1.0 && s >= 0.0 && s <= len) {
                        best = tt
                        nx = (mpx + tt * dpx) / r; ny = (mpy + tt * dpy) / r; nz = (mpz + tt * dpz) / r
                    }
                }
            }
        }
        best = sphere(px, py, pz, dx, dy, dz, ax, ay, az, r, best)
        best = sphere(px, py, pz, dx, dy, dz, bx, by, bz, r, best)
        if (best > 1.0) return false
        t = best
        return true
    }

    /** First entry into the sphere (cx, cy, cz, r) when earlier than [best]; returns the new best. */
    private fun sphere(px: Double, py: Double, pz: Double, dx: Double, dy: Double, dz: Double, cx: Double, cy: Double, cz: Double, r: Double, best: Double): Double {
        val fx = px - cx; val fy = py - cy; val fz = pz - cz
        val b = fx * dx + fy * dy + fz * dz
        if (b >= 0.0) return best
        val a = dx * dx + dy * dy + dz * dz
        val c = fx * fx + fy * fy + fz * fz - r * r
        val disc = b * b - a * c
        if (disc < 0.0) return best
        val tt = (-b - DMath.sqrt(disc)) / a
        if (tt < 0.0 || tt > 1.0 || tt >= best) return best
        nx = (fx + tt * dx) / r; ny = (fy + tt * dy) / r; nz = (fz + tt * dz) / r
        return tt
    }
}
