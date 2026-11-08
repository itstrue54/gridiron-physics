package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.math.DMath

/**
 * Narrow phase: one function per shape pair, named after the argument order it takes. [collide] dispatches
 * any order and flips the normal for the reversed pairs, so the normal always points from `a` to `b`.
 * Points are reported while the shapes are within [margin] of touching (speculative contacts).
 *
 * Stateless: scratch values live in the caller's [Manifold], so separate Worlds on separate threads never
 * share state, and nothing allocates.
 */
internal object Collide {
    /** Base speculative distance (m). The world adds each pair's possible closing distance per tick. */
    const val SPECULATIVE = 0.02

    /** How close the SAT axis must be to the true core distance to count as a face contact. */
    private const val FACE_TOL = 1e-7

    private const val ID_VERTEX = 0x40000000
    private const val ID_FLIP = 0x01000000

    fun collide(a: Geom, b: Geom, m: Manifold, margin: Double) {
        val ka = a.kind
        val kb = b.kind
        when (ka) {
            Geom.CIRCLE -> when (kb) {
                Geom.CIRCLE -> circleCircle(a, b, m, margin)
                Geom.CAPSULE -> circleCapsule(a, b, m, margin)
                Geom.POLYGON -> circlePolygon(a, b, m, margin)
                else -> circleSegment(a, b, m, margin)
            }
            Geom.CAPSULE -> when (kb) {
                Geom.CIRCLE -> { circleCapsule(b, a, m, margin); m.flip() }
                Geom.CAPSULE -> capsuleCapsule(a, b, m, margin)
                Geom.POLYGON -> capsulePolygon(a, b, m, margin)
                else -> capsuleSegment(a, b, m, margin)
            }
            Geom.POLYGON -> when (kb) {
                Geom.CIRCLE -> { circlePolygon(b, a, m, margin); m.flip() }
                Geom.CAPSULE -> { capsulePolygon(b, a, m, margin); m.flip() }
                Geom.POLYGON -> polygonPolygon(a, b, m, margin)
                else -> roundedPolygons(a, b, m, margin)
            }
            else -> when (kb) {
                Geom.CIRCLE -> { circleSegment(b, a, m, margin); m.flip() }
                Geom.CAPSULE -> { capsuleSegment(b, a, m, margin); m.flip() }
                Geom.POLYGON -> roundedPolygons(a, b, m, margin)
                else -> m.count = 0 // segments are static; static pairs never collide
            }
        }
    }

    fun circleCircle(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) {
        // Concentric circles have no natural normal. Tie-break on radius so swapping the arguments flips it
        // (identical concentric circles are the one case that cannot be made order-independent).
        val fx = if (a.radius <= b.radius) 1.0 else -1.0
        circles(a.vx[0], a.vy[0], a.radius, b.vx[0], b.vy[0], b.radius, fx, 0.0, m, margin)
    }

    fun circleCapsule(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = circleCore2(a, b, m, margin)

    fun circleSegment(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = circleCore2(a, b, m, margin)

    fun circlePolygon(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) {
        polygonCircle(b, a, m, margin)
        m.flip()
    }

    fun capsuleCapsule(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = roundedPolygons(a, b, m, margin)

    fun capsulePolygon(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = roundedPolygons(a, b, m, margin)

    fun polygonPolygon(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = roundedPolygons(a, b, m, margin)

    fun capsuleSegment(a: Geom, b: Geom, m: Manifold, margin: Double = 0.0) = roundedPolygons(a, b, m, margin)

    // ------------------------------------------------------------ circles

    /** Two circles (or a circle and a closest core point). The fallback normal is used for coincident centers. */
    private fun circles(
        ax: Double, ay: Double, ra: Double, bx: Double, by: Double, rb: Double,
        fallbackNx: Double, fallbackNy: Double, m: Manifold, margin: Double,
    ) {
        val dx = bx - ax
        val dy = by - ay
        val d = DMath.hypot2(dx, dy)
        val sep = d - ra - rb
        if (sep > margin) { m.count = 0; return }
        val nx: Double
        val ny: Double
        if (d < 1e-12) { nx = fallbackNx; ny = fallbackNy } else { nx = dx / d; ny = dy / d }
        m.count = 1
        m.nx = nx; m.ny = ny
        m.px[0] = 0.5 * (ax + nx * ra + bx - nx * rb)
        m.py[0] = 0.5 * (ay + ny * ra + by - ny * rb)
        m.sep[0] = sep
        m.id[0] = 0
    }

    /** Circle `a` against a 2-vertex core `b` (capsule or segment). */
    private fun circleCore2(a: Geom, b: Geom, m: Manifold, margin: Double) {
        val cx = a.vx[0]; val cy = a.vy[0]
        val x1 = b.vx[0]; val y1 = b.vy[0]
        val ex = b.vx[1] - x1; val ey = b.vy[1] - y1
        val t = DMath.clamp(((cx - x1) * ex + (cy - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
        val qx = x1 + t * ex; val qy = y1 + t * ey
        // Coincident: push the circle out along the core's first normal (B lies on its far side).
        circles(cx, cy, a.radius, qx, qy, b.radius, -b.nx[0], -b.ny[0], m, margin)
    }

    /** Polygon `p` (any core with ≥ 3 vertices, radius may be > 0) against circle `c`; normal from p to c. */
    private fun polygonCircle(p: Geom, c: Geom, m: Manifold, margin: Double) {
        val cx = c.vx[0]; val cy = c.vy[0]
        val n = p.count
        var best = -Double.MAX_VALUE
        var e = 0
        for (i in 0 until n) {
            val s = p.nx[i] * (cx - p.vx[i]) + p.ny[i] * (cy - p.vy[i])
            if (s > best) { best = s; e = i }
        }
        val rSum = p.radius + c.radius
        if (best - rSum > margin) { m.count = 0; return }
        val i2 = if (e + 1 == n) 0 else e + 1
        val v1x = p.vx[e]; val v1y = p.vy[e]
        val v2x = p.vx[i2]; val v2y = p.vy[i2]
        if (best > 0.0) {
            val u1 = (cx - v1x) * (v2x - v1x) + (cy - v1y) * (v2y - v1y)
            val u2 = (cx - v2x) * (v1x - v2x) + (cy - v2y) * (v1y - v2y)
            if (u1 < 0.0) { circles(v1x, v1y, p.radius, cx, cy, c.radius, p.nx[e], p.ny[e], m, margin); m.id[0] = ID_VERTEX or e; return }
            if (u2 < 0.0) { circles(v2x, v2y, p.radius, cx, cy, c.radius, p.nx[e], p.ny[e], m, margin); m.id[0] = ID_VERTEX or i2; return }
        }
        // Face region (or center inside the polygon).
        val nx = p.nx[e]; val ny = p.ny[e]
        m.count = 1
        m.nx = nx; m.ny = ny
        val fx = cx - nx * best; val fy = cy - ny * best // projection of the center on the face
        m.px[0] = 0.5 * (fx + nx * p.radius + cx - nx * c.radius)
        m.py[0] = 0.5 * (fy + ny * p.radius + cy - ny * c.radius)
        m.sep[0] = best - rSum
        m.id[0] = e
    }

    // ------------------------------------------------------------ rounded convex polygons (SAT + clipping)

    /** Largest separation of `q`'s core from any edge of `p`'s core; the edge index goes to `m.satEdge`. */
    private fun maxSeparation(p: Geom, q: Geom, m: Manifold): Double {
        var best = -Double.MAX_VALUE
        var edge = 0
        for (i in 0 until p.count) {
            val nx = p.nx[i]; val ny = p.ny[i]
            val ox = p.vx[i]; val oy = p.vy[i]
            var si = Double.MAX_VALUE
            for (j in 0 until q.count) {
                val d = nx * (q.vx[j] - ox) + ny * (q.vy[j] - oy)
                if (d < si) si = d
            }
            if (si > best) { best = si; edge = i }
        }
        m.satEdge = edge
        return best
    }

    /** Distance between the cores of `a` and `b` (assumed not overlapping); closest points go to m.cpA / m.cpB. */
    private fun coreDistance(a: Geom, b: Geom, m: Manifold): Double {
        var best = Double.MAX_VALUE
        // Vertices of b against edges of a, then vertices of a against edges of b.
        for (pass in 0..1) {
            val e = if (pass == 0) a else b
            val v = if (pass == 0) b else a
            val edges = if (e.count == 2) 1 else e.count
            for (i in 0 until edges) {
                val x1 = e.vx[i]; val y1 = e.vy[i]
                val i2 = if (i + 1 == e.count) 0 else i + 1
                val ex = e.vx[i2] - x1; val ey = e.vy[i2] - y1
                val ee = ex * ex + ey * ey
                for (j in 0 until v.count) {
                    val px = v.vx[j]; val py = v.vy[j]
                    val t = DMath.clamp(((px - x1) * ex + (py - y1) * ey) / ee, 0.0, 1.0)
                    val qx = x1 + t * ex; val qy = y1 + t * ey
                    val dx = px - qx; val dy = py - qy
                    val d2 = dx * dx + dy * dy
                    if (d2 < best) {
                        best = d2
                        if (pass == 0) { m.cpAx = qx; m.cpAy = qy; m.cpBx = px; m.cpBy = py } else { m.cpAx = px; m.cpAy = py; m.cpBx = qx; m.cpBy = qy }
                    }
                }
            }
        }
        return DMath.sqrt(best)
    }

    /**
     * Any two shapes with a 2+ vertex core: capsule, segment, polygon. SAT picks the reference face; the
     * incident edge is clipped against it, giving up to 2 points. When the cores are apart and the closest
     * features are not a face (corner to corner, capsule end to end), a single point along the true closest
     * direction is used instead.
     */
    private fun roundedPolygons(a: Geom, b: Geom, m: Manifold, margin: Double) {
        m.count = 0
        val rA = a.radius
        val rB = b.radius
        val rSum = rA + rB
        val sepA = maxSeparation(a, b, m)
        val edgeA = m.satEdge
        if (sepA > rSum + margin) return
        val sepB = maxSeparation(b, a, m)
        val edgeB = m.satEdge
        if (sepB > rSum + margin) return
        val coreSep = if (sepA > sepB) sepA else sepB

        if (coreSep > -1e-9) {
            val d = coreDistance(a, b, m)
            val cpAx = m.cpAx; val cpAy = m.cpAy; val cpBx = m.cpBx; val cpBy = m.cpBy
            if (d > 1e-9) {
                if (d - rSum > margin) return
                if (coreSep < d - FACE_TOL) {
                    // Vertex contact: normal along the closest points.
                    val nx = (cpBx - cpAx) / d
                    val ny = (cpBy - cpAy) / d
                    m.count = 1
                    m.nx = nx; m.ny = ny
                    m.px[0] = 0.5 * (cpAx + nx * rA + cpBx - nx * rB)
                    m.py[0] = 0.5 * (cpAy + ny * rA + cpBy - ny * rB)
                    m.sep[0] = d - rSum
                    m.id[0] = ID_VERTEX
                    return
                }
            }
        }

        // Face contact. Reference = the shape with the larger separation (first argument on an exact tie).
        val flip = sepB > sepA
        val ref = if (flip) b else a
        val inc = if (flip) a else b
        val e = if (flip) edgeB else edgeA
        val rRef = if (flip) rB else rA
        val rInc = if (flip) rA else rB
        val i2 = if (e + 1 == ref.count) 0 else e + 1
        val nx = ref.nx[e]; val ny = ref.ny[e]
        val v1x = ref.vx[e]; val v1y = ref.vy[e]
        var tx = ref.vx[i2] - v1x; var ty = ref.vy[i2] - v1y
        val len = DMath.hypot2(tx, ty)
        tx /= len; ty /= len

        // Incident edge: the one most anti-parallel to the reference normal.
        var k = 0
        var minDot = Double.MAX_VALUE
        for (i in 0 until inc.count) {
            val dot = inc.nx[i] * nx + inc.ny[i] * ny
            if (dot < minDot) { minDot = dot; k = i }
        }
        val k2 = if (k + 1 == inc.count) 0 else k + 1
        val w1x = inc.vx[k]; val w1y = inc.vy[k]
        val w2x = inc.vx[k2]; val w2y = inc.vy[k2]
        val ua = (w1x - v1x) * tx + (w1y - v1y) * ty
        val ub = (w2x - v1x) * tx + (w2y - v1y) * ty

        val idBase = (if (flip) ID_FLIP else 0) or (e shl 16) or (k shl 8)
        if (!((ua < 0.0 && ub < 0.0) || (ua > len && ub > len))) {
            // Clip each incident vertex into the reference face's [0, len] slab.
            for (j in 0..1) {
                val u = if (j == 0) ua else ub
                val uo = if (j == 0) ub else ua
                val sx = if (j == 0) w1x else w2x; val sy = if (j == 0) w1y else w2y
                val ox = if (j == 0) w2x else w1x; val oy = if (j == 0) w2y else w1y
                var qx = sx; var qy = sy
                if (u < 0.0) {
                    val f = -u / (uo - u); qx = sx + f * (ox - sx); qy = sy + f * (oy - sy)
                } else if (u > len) {
                    val f = (len - u) / (uo - u); qx = sx + f * (ox - sx); qy = sy + f * (oy - sy)
                }
                addFacePoint(m, qx, qy, v1x, v1y, nx, ny, rRef, rInc, margin, idBase or j)
            }
        }
        if (m.count == 0 && coreSep < 0.0) {
            // Clipping found nothing for overlapping cores (degenerate geometry): use the deepest incident vertex.
            var deep = 0
            var deepS = Double.MAX_VALUE
            for (i in 0 until inc.count) {
                val s = (inc.vx[i] - v1x) * nx + (inc.vy[i] - v1y) * ny
                if (s < deepS) { deepS = s; deep = i }
            }
            addFacePoint(m, inc.vx[deep], inc.vy[deep], v1x, v1y, nx, ny, rRef, rInc, margin, idBase or 0xFF)
        }
        if (m.count == 0) return
        m.nx = if (flip) -nx else nx
        m.ny = if (flip) -ny else ny
    }

    private fun addFacePoint(
        m: Manifold, qx: Double, qy: Double, v1x: Double, v1y: Double, nx: Double, ny: Double,
        rRef: Double, rInc: Double, margin: Double, id: Int,
    ) {
        val cs = (qx - v1x) * nx + (qy - v1y) * ny
        val sep = cs - rRef - rInc
        if (sep > margin) return
        val i = m.count
        // Midway between the reference surface (projection + rRef) and the incident surface (q − rInc).
        m.px[i] = qx + nx * (0.5 * (rRef - cs - rInc))
        m.py[i] = qy + ny * (0.5 * (rRef - cs - rInc))
        m.sep[i] = sep
        m.id[i] = id
        m.count = i + 1
    }
}
