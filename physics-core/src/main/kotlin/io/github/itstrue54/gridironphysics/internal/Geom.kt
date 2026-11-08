package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.math.DMath

/**
 * A shape placed in the world, cached once per tick. Every shape is stored as a rounded convex polygon:
 * a circle is 1 core vertex plus a radius, a capsule 2 vertices plus a radius, a segment 2 vertices with
 * radius 0, a polygon 3..8 vertices with radius 0. A 2-vertex core has two edges (v0→v1 and v1→v0) whose
 * normals point to either side.
 */
internal class Geom {
    @JvmField var kind = CIRCLE
    @JvmField var radius = 0.0
    @JvmField var count = 0
    @JvmField val vx = DoubleArray(ShapeDef.MAX_VERTICES)
    @JvmField val vy = DoubleArray(ShapeDef.MAX_VERTICES)
    @JvmField val nx = DoubleArray(ShapeDef.MAX_VERTICES)
    @JvmField val ny = DoubleArray(ShapeDef.MAX_VERTICES)
    @JvmField var minX = 0.0
    @JvmField var minY = 0.0
    @JvmField var maxX = 0.0
    @JvmField var maxY = 0.0

    fun set(shape: ShapeDef, x: Double, y: Double, angle: Double) {
        val c = DMath.cos(angle)
        val s = DMath.sin(angle)
        when (shape) {
            is ShapeDef.Circle -> {
                kind = CIRCLE; radius = shape.radius; count = 1
                vx[0] = x; vy[0] = y
            }
            is ShapeDef.Capsule -> {
                kind = CAPSULE; radius = shape.radius
                val h = shape.halfLength
                setSegment(x - c * h, y - s * h, x + c * h, y + s * h)
            }
            is ShapeDef.Segment -> {
                kind = SEGMENT; radius = 0.0
                setSegment(
                    x + c * shape.x1 - s * shape.y1, y + s * shape.x1 + c * shape.y1,
                    x + c * shape.x2 - s * shape.y2, y + s * shape.x2 + c * shape.y2,
                )
            }
            is ShapeDef.Polygon -> {
                kind = POLYGON; radius = 0.0; count = shape.count
                for (i in 0 until count) {
                    val lx = shape.vertex(2 * i); val ly = shape.vertex(2 * i + 1)
                    vx[i] = x + c * lx - s * ly; vy[i] = y + s * lx + c * ly
                    val lnx = shape.normal(2 * i); val lny = shape.normal(2 * i + 1)
                    nx[i] = c * lnx - s * lny; ny[i] = s * lnx + c * lny
                }
            }
        }
        var x0 = vx[0]; var x1 = vx[0]; var y0 = vy[0]; var y1 = vy[0]
        for (i in 1 until count) {
            if (vx[i] < x0) x0 = vx[i]; if (vx[i] > x1) x1 = vx[i]
            if (vy[i] < y0) y0 = vy[i]; if (vy[i] > y1) y1 = vy[i]
        }
        minX = x0 - radius; maxX = x1 + radius; minY = y0 - radius; maxY = y1 + radius
    }

    private fun setSegment(x1: Double, y1: Double, x2: Double, y2: Double) {
        count = 2
        vx[0] = x1; vy[0] = y1; vx[1] = x2; vy[1] = y2
        val ex = x2 - x1
        val ey = y2 - y1
        val len = DMath.hypot2(ex, ey)
        nx[0] = ey / len; ny[0] = -ex / len
        nx[1] = -nx[0]; ny[1] = -ny[0]
    }

    companion object {
        const val CIRCLE = 0
        const val CAPSULE = 1
        const val POLYGON = 2
        const val SEGMENT = 3
    }
}

/**
 * Narrow-phase output: a unit normal pointing from shape A to shape B, and up to 2 points. Each point sits
 * midway between the two surfaces; `sep` is the signed distance between them (negative when overlapping, so
 * penetration depth is `-sep`). `id` identifies the features that made the point, for warm starting.
 */
internal class Manifold {
    @JvmField var count = 0
    @JvmField var nx = 0.0
    @JvmField var ny = 0.0
    @JvmField val px = DoubleArray(2)
    @JvmField val py = DoubleArray(2)
    @JvmField val sep = DoubleArray(2)
    @JvmField val id = IntArray(2)

    // Narrow-phase scratch (kept here so Collide holds no state).
    @JvmField var satEdge = 0
    @JvmField var cpAx = 0.0
    @JvmField var cpAy = 0.0
    @JvmField var cpBx = 0.0
    @JvmField var cpBy = 0.0

    fun flip() {
        nx = -nx; ny = -ny
    }
}
