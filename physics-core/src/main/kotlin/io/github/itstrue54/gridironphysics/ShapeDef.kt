package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath

/**
 * Collision shape of a body, in the body's local frame (the body origin is its center of mass).
 * Immutable, so one instance can be shared by many bodies. Any shape can be a [sensor]: it reports
 * overlaps but never pushes.
 */
sealed class ShapeDef(val sensor: Boolean) {

    /** Upright players and the ball on the ground. Centered on the body origin. */
    class Circle(val radius: Double, sensor: Boolean = false) : ShapeDef(sensor) {
        init {
            require(radius.isFinite() && radius > 0.0) { "Circle radius must be finite and > 0: $radius" }
        }

        override fun toString(): String = "Circle(r=$radius${if (sensor) ", sensor" else ""})"
    }

    /**
     * Diving, falling and downed players: a segment from (−halfLength, 0) to (+halfLength, 0) along the
     * body's local x axis (so it follows the body angle), swept by [radius].
     */
    class Capsule(val halfLength: Double, val radius: Double, sensor: Boolean = false) : ShapeDef(sensor) {
        init {
            require(halfLength.isFinite() && halfLength > 0.0) { "Capsule halfLength must be finite and > 0: $halfLength" }
            require(radius.isFinite() && radius > 0.0) { "Capsule radius must be finite and > 0: $radius" }
        }

        override fun toString(): String = "Capsule(h=$halfLength, r=$radius${if (sensor) ", sensor" else ""})"
    }

    /**
     * Convex polygon with 3 to [MAX_VERTICES] vertices, given as interleaved local `x, y` pairs. The input is
     * copied; clockwise input is reordered to counter-clockwise. Concave, collinear or non-finite input is
     * rejected.
     */
    class Polygon(vertices: DoubleArray, sensor: Boolean = false) : ShapeDef(sensor) {
        private val v: DoubleArray

        /** Outward unit edge normals: normal `i` belongs to the edge from vertex `i` to vertex `i + 1`. */
        private val n: DoubleArray

        init {
            require(vertices.size % 2 == 0) { "Polygon needs x, y pairs" }
            val k = vertices.size / 2
            require(k in 3..MAX_VERTICES) { "Polygon needs 3..$MAX_VERTICES vertices: $k" }
            require(vertices.all { it.isFinite() }) { "Polygon vertices must be finite" }
            var area2 = 0.0
            for (i in 0 until k) {
                val j = (i + 1) % k
                area2 += vertices[2 * i] * vertices[2 * j + 1] - vertices[2 * j] * vertices[2 * i + 1]
            }
            v = DoubleArray(2 * k)
            for (i in 0 until k) {
                val src = if (area2 > 0.0) i else k - 1 - i
                v[2 * i] = vertices[2 * src]; v[2 * i + 1] = vertices[2 * src + 1]
            }
            n = DoubleArray(2 * k)
            for (i in 0 until k) {
                val j = (i + 1) % k
                val ex = v[2 * j] - v[2 * i]
                val ey = v[2 * j + 1] - v[2 * i + 1]
                val len = DMath.hypot2(ex, ey)
                require(len > 1e-6) { "Polygon has a degenerate edge at vertex $i" }
                n[2 * i] = ey / len; n[2 * i + 1] = -ex / len
                // Strict convexity: the next vertex must turn left.
                val l = (j + 1) % k
                val cross = ex * (v[2 * l + 1] - v[2 * j + 1]) - ey * (v[2 * l] - v[2 * j])
                require(cross > 1e-9 * len) { "Polygon must be strictly convex (vertex ${j})" }
            }
        }

        /** Number of vertices. */
        val count: Int get() = v.size / 2

        /** Interleaved coordinate `i` of the counter-clockwise vertex list (x at even, y at odd indexes). */
        fun vertex(i: Int): Double = v[i]

        /** Interleaved coordinate `i` of the outward edge normal list. */
        fun normal(i: Int): Double = n[i]

        override fun toString(): String = "Polygon(${count} vertices${if (sensor) ", sensor" else ""})"
    }

    /** A line segment between two local points. Static bodies only: runoff walls, line sensors. */
    class Segment(val x1: Double, val y1: Double, val x2: Double, val y2: Double, sensor: Boolean = false) : ShapeDef(sensor) {
        init {
            require(x1.isFinite() && y1.isFinite() && x2.isFinite() && y2.isFinite()) { "Segment points must be finite" }
            require(DMath.hypot2(x2 - x1, y2 - y1) > 1e-6) { "Segment must have length" }
        }

        override fun toString(): String = "Segment(($x1, $y1)-($x2, $y2)${if (sensor) ", sensor" else ""})"
    }

    companion object {
        const val MAX_VERTICES = 8

        /** Axis-aligned box centered on the body origin, rotated with the body. */
        fun box(halfWidth: Double, halfHeight: Double, sensor: Boolean = false): Polygon {
            require(halfWidth > 0.0 && halfHeight > 0.0) { "box half sizes must be > 0" }
            return Polygon(
                doubleArrayOf(-halfWidth, -halfHeight, halfWidth, -halfHeight, halfWidth, halfHeight, -halfWidth, halfHeight),
                sensor,
            )
        }
    }
}
