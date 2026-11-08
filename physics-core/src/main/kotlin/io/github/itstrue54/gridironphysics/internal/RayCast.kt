package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.math.DMath

/** Output of [RayCast.cast]: where along the ray (0..1) and the outward surface normal there. */
internal class RayResult {
    @JvmField var t = 0.0
    @JvmField var nx = 0.0
    @JvmField var ny = 0.0
}

/**
 * Ray against a [Geom] grown by an extra radius (0 for a raycast, the cast radius for a circle cast). Every Geom is
 * a rounded convex core, and growing it keeps it one: the surface is the core's edges pushed out along their
 * normals by the total radius, joined by arcs around the core vertices. Because each vertex circle lies inside the
 * shape, the first point where the ray enters any edge piece or vertex circle is the first point where it enters
 * the shape. Stateless and allocation-free.
 */
internal object RayCast {

    /**
     * Casts p(t) = (ox, oy) + t·(dx, dy), t ∈ [0, 1]. Returns true and fills [r] on a hit. A ray that starts
     * inside the shape hits at t = 0 with the normal pointing back along the ray.
     */
    fun cast(g: Geom, extra: Double, ox: Double, oy: Double, dx: Double, dy: Double, r: RayResult): Boolean {
        val rad = g.radius + extra
        val n = g.count
        if (startsInside(g, rad, ox, oy)) {
            val len = DMath.hypot2(dx, dy)
            r.t = 0.0; r.nx = -dx / len; r.ny = -dy / len
            return true
        }
        var best = Double.MAX_VALUE
        if (n == 1) {
            best = circle(g.vx[0], g.vy[0], rad, ox, oy, dx, dy, best, r)
        } else {
            for (i in 0 until n) {
                if (rad > 0.0) best = circle(g.vx[i], g.vy[i], rad, ox, oy, dx, dy, best, r)
                val j = if (i + 1 == n) 0 else i + 1
                val nx = g.nx[i]; val ny = g.ny[i]
                val dn = dx * nx + dy * ny
                if (dn >= 0.0) continue // parallel, or leaving through this side
                val px = g.vx[i] + rad * nx; val py = g.vy[i] + rad * ny
                val t = ((px - ox) * nx + (py - oy) * ny) / dn
                if (t < 0.0 || t > 1.0 || t >= best) continue
                val ex = g.vx[j] - g.vx[i]; val ey = g.vy[j] - g.vy[i]
                val u = ((ox + t * dx - px) * ex + (oy + t * dy - py) * ey) / (ex * ex + ey * ey)
                if (u < 0.0 || u > 1.0) continue
                best = t; r.t = t; r.nx = nx; r.ny = ny
            }
        }
        return best != Double.MAX_VALUE
    }

    /** Entry into the circle (cx, cy, rad) when earlier than [best]; returns the new best. */
    private fun circle(cx: Double, cy: Double, rad: Double, ox: Double, oy: Double, dx: Double, dy: Double, best: Double, r: RayResult): Double {
        val fx = ox - cx; val fy = oy - cy
        val a = dx * dx + dy * dy
        val b = fx * dx + fy * dy
        if (b >= 0.0) return best // moving away (the start is outside, so no entry ahead)
        val c = fx * fx + fy * fy - rad * rad
        val disc = b * b - a * c
        if (disc < 0.0) return best
        val t = (-b - DMath.sqrt(disc)) / a
        if (t < 0.0 || t > 1.0 || t >= best) return best
        r.t = t
        r.nx = (fx + t * dx) / rad; r.ny = (fy + t * dy) / rad
        return t
    }

    private fun startsInside(g: Geom, rad: Double, ox: Double, oy: Double): Boolean {
        val n = g.count
        if (n >= 3) {
            var s = -Double.MAX_VALUE
            for (i in 0 until n) s = DMath.max(s, g.nx[i] * (ox - g.vx[i]) + g.ny[i] * (oy - g.vy[i]))
            if (s < 0.0) return true
            if (rad == 0.0) return false
        }
        if (rad == 0.0) return false
        var d2 = Double.MAX_VALUE
        if (n == 1) {
            val fx = ox - g.vx[0]; val fy = oy - g.vy[0]
            d2 = fx * fx + fy * fy
        } else {
            val edges = if (n == 2) 1 else n
            for (i in 0 until edges) {
                val j = if (i + 1 == n) 0 else i + 1
                d2 = DMath.min(d2, segmentDist2(g.vx[i], g.vy[i], g.vx[j], g.vy[j], ox, oy))
            }
        }
        return d2 < rad * rad
    }

    private fun segmentDist2(x1: Double, y1: Double, x2: Double, y2: Double, px: Double, py: Double): Double {
        val ex = x2 - x1; val ey = y2 - y1
        val u = DMath.clamp(((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
        val qx = x1 + u * ex - px; val qy = y1 + u * ey - py
        return qx * qx + qy * qy
    }
}
