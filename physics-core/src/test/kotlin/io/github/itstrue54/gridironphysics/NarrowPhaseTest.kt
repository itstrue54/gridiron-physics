package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.Collide
import io.github.itstrue54.gridironphysics.internal.Geom
import io.github.itstrue54.gridironphysics.internal.Manifold
import io.github.itstrue54.gridironphysics.math.SplitMix64
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Every narrow-phase pair, with known values, in both argument orders (CLAUDE.md rule), plus property
 * tests on random shape pairs: the normal is unit length and penetration is >= 0 when overlapping.
 */
class NarrowPhaseTest {
    private fun g(shape: ShapeDef, x: Double = 0.0, y: Double = 0.0, angle: Double = 0.0) = Geom().also { it.set(shape, x, y, angle) }

    private data class P(val x: Double, val y: Double, val depth: Double)

    private fun points(m: Manifold): List<P> =
        (0 until m.count).map { P(m.px[it], m.py[it], -m.sep[it]) }.sortedWith(compareBy({ it.x }, { it.y }))

    private fun run(f: (Geom, Geom, Manifold) -> Unit, a: Geom, b: Geom): Manifold = Manifold().also { f(a, b, it) }

    /**
     * Runs the named pair function on (a, b) and the dispatcher on (b, a), checks they agree (same points,
     * opposite normals), and returns the (a, b) manifold.
     */
    private fun bothOrders(pair: (Geom, Geom, Manifold) -> Unit, a: Geom, b: Geom): Manifold {
        val ab = run(pair, a, b)
        val ab2 = run({ x, y, m -> Collide.collide(x, y, m, 0.0) }, a, b)
        val ba = run({ x, y, m -> Collide.collide(x, y, m, 0.0) }, b, a)
        assertEquals(ab.count, ab2.count, "dispatcher (a, b) count")
        assertEquals(ab.count, ba.count, "count in both orders")
        if (ab.count > 0) {
            assertEquals(ab.nx, -ba.nx, 1e-12, "nx flips"); assertEquals(ab.ny, -ba.ny, 1e-12, "ny flips")
            assertEquals(ab.nx, ab2.nx, 1e-12); assertEquals(ab.ny, ab2.ny, 1e-12)
            val p1 = points(ab); val p2 = points(ba)
            p1.zip(p2).forEach { (u, v) ->
                assertEquals(u.x, v.x, 1e-9); assertEquals(u.y, v.y, 1e-9); assertEquals(u.depth, v.depth, 1e-9)
            }
        }
        return ab
    }

    private fun assertPoint(m: Manifold, i: Int, x: Double, y: Double, depth: Double) {
        val p = points(m)[i]
        assertEquals(x, p.x, 1e-9, "x"); assertEquals(y, p.y, 1e-9, "y"); assertEquals(depth, p.depth, 1e-9, "depth")
    }

    private fun assertNormal(m: Manifold, nx: Double, ny: Double) {
        assertEquals(nx, m.nx, 1e-9, "nx"); assertEquals(ny, m.ny, 1e-9, "ny")
    }

    // ------------------------------------------------------------ circle–circle

    @Test fun circleCircle() {
        val m = bothOrders(Collide::circleCircle, g(ShapeDef.Circle(1.0)), g(ShapeDef.Circle(1.0), 1.5, 0.0))
        assertEquals(1, m.count); assertNormal(m, 1.0, 0.0); assertPoint(m, 0, 0.75, 0.0, 0.5)
    }

    @Test fun circleCircleSeparated() {
        assertEquals(0, bothOrders(Collide::circleCircle, g(ShapeDef.Circle(1.0)), g(ShapeDef.Circle(1.0), 2.1, 0.0)).count)
    }

    @Test fun circleCircleConcentricHasUnitNormal() {
        val m = bothOrders(Collide::circleCircle, g(ShapeDef.Circle(0.5)), g(ShapeDef.Circle(0.3)))
        assertEquals(1, m.count); assertEquals(1.0, hypot(m.nx, m.ny), 1e-12); assertEquals(0.8, -m.sep[0], 1e-12)
    }

    // ------------------------------------------------------------ circle–capsule

    @Test fun circleCapsuleSide() {
        val m = bothOrders(Collide::circleCapsule, g(ShapeDef.Circle(0.5), 0.5, 0.9), g(ShapeDef.Capsule(1.0, 0.5)))
        assertEquals(1, m.count); assertNormal(m, 0.0, -1.0); assertPoint(m, 0, 0.5, 0.45, 0.1)
    }

    @Test fun circleCapsuleEndCapRotated() {
        // Capsule along +y (angle 90°): its top cap center is at (0, 1).
        val m = bothOrders(Collide::circleCapsule, g(ShapeDef.Circle(0.5), 0.0, 1.9), g(ShapeDef.Capsule(1.0, 0.5), angle = PI / 2))
        assertEquals(1, m.count); assertNormal(m, 0.0, -1.0); assertPoint(m, 0, 0.0, 1.45, 0.1)
    }

    @Test fun circleCapsuleSeparated() {
        assertEquals(0, bothOrders(Collide::circleCapsule, g(ShapeDef.Circle(0.5), 0.0, 1.1), g(ShapeDef.Capsule(1.0, 0.5))).count)
    }

    // ------------------------------------------------------------ capsule–capsule

    @Test fun capsuleCapsuleParallelGivesTwoPoints() {
        val m = bothOrders(Collide::capsuleCapsule, g(ShapeDef.Capsule(1.0, 0.5)), g(ShapeDef.Capsule(1.0, 0.5), 0.5, 0.9))
        assertEquals(2, m.count); assertNormal(m, 0.0, 1.0)
        assertPoint(m, 0, -0.5, 0.45, 0.1); assertPoint(m, 1, 1.0, 0.45, 0.1)
    }

    @Test fun capsuleCapsuleCrossed() {
        val m = bothOrders(Collide::capsuleCapsule, g(ShapeDef.Capsule(1.0, 0.5)), g(ShapeDef.Capsule(1.0, 0.5), 0.2, 1.9, PI / 2))
        assertEquals(1, m.count); assertNormal(m, 0.0, 1.0); assertPoint(m, 0, 0.2, 0.45, 0.1)
    }

    @Test fun capsuleCapsuleEndToEnd() {
        val m = bothOrders(Collide::capsuleCapsule, g(ShapeDef.Capsule(1.0, 0.5)), g(ShapeDef.Capsule(1.0, 0.5), 2.9, 0.0))
        assertEquals(1, m.count); assertNormal(m, 1.0, 0.0); assertPoint(m, 0, 1.45, 0.0, 0.1)
    }

    @Test fun capsuleCapsuleSeparated() {
        assertEquals(0, bothOrders(Collide::capsuleCapsule, g(ShapeDef.Capsule(1.0, 0.5)), g(ShapeDef.Capsule(1.0, 0.5), 0.0, 1.01)).count)
        assertEquals(0, bothOrders(Collide::capsuleCapsule, g(ShapeDef.Capsule(1.0, 0.5)), g(ShapeDef.Capsule(1.0, 0.5), 3.01, 0.0)).count)
    }

    // ------------------------------------------------------------ circle–polygon

    @Test fun circlePolygonFace() {
        val m = bothOrders(Collide::circlePolygon, g(ShapeDef.Circle(0.5), 1.4, 0.0), g(ShapeDef.box(1.0, 1.0)))
        assertEquals(1, m.count); assertNormal(m, -1.0, 0.0); assertPoint(m, 0, 0.95, 0.0, 0.1)
    }

    @Test fun circlePolygonCorner() {
        val m = bothOrders(Collide::circlePolygon, g(ShapeDef.Circle(0.5), 1.3, 1.3), g(ShapeDef.box(1.0, 1.0)))
        val s = 1.0 / sqrt(2.0)
        val d = 0.3 * sqrt(2.0)
        assertEquals(1, m.count); assertNormal(m, -s, -s)
        assertEquals(0.5 - d, -m.sep[0], 1e-9)
    }

    @Test fun circlePolygonCenterInside() {
        val m = bothOrders(Collide::circlePolygon, g(ShapeDef.Circle(0.25), 0.0, 0.8), g(ShapeDef.box(2.0, 1.0)))
        assertEquals(1, m.count); assertNormal(m, 0.0, -1.0); assertEquals(0.45, -m.sep[0], 1e-9)
    }

    @Test fun circlePolygonSeparated() {
        assertEquals(0, bothOrders(Collide::circlePolygon, g(ShapeDef.Circle(0.5), 1.36, 1.36), g(ShapeDef.box(1.0, 1.0))).count)
    }

    // ------------------------------------------------------------ capsule–polygon

    @Test fun capsulePolygonLyingOnFace() {
        val m = bothOrders(Collide::capsulePolygon, g(ShapeDef.Capsule(0.5, 0.5), 0.0, 1.4), g(ShapeDef.box(1.0, 1.0)))
        assertEquals(2, m.count); assertNormal(m, 0.0, -1.0)
        assertPoint(m, 0, -0.5, 0.95, 0.1); assertPoint(m, 1, 0.5, 0.95, 0.1)
    }

    @Test fun capsulePolygonEndCapIntoCorner() {
        // Capsule along x, its right cap center at (−1.3, 1.3); box corner at (−1, 1).
        val m = bothOrders(Collide::capsulePolygon, g(ShapeDef.Capsule(0.5, 0.5), -1.8, 1.3), g(ShapeDef.box(1.0, 1.0)))
        val s = 1.0 / sqrt(2.0)
        assertEquals(1, m.count); assertNormal(m, s, -s); assertEquals(0.5 - 0.3 * sqrt(2.0), -m.sep[0], 1e-9)
    }

    @Test fun capsulePolygonSeparated() {
        assertEquals(0, bothOrders(Collide::capsulePolygon, g(ShapeDef.Capsule(0.5, 0.5), 0.0, 1.51), g(ShapeDef.box(1.0, 1.0))).count)
    }

    // ------------------------------------------------------------ polygon–polygon

    @Test fun polygonPolygonFaceToFace() {
        val m = bothOrders(Collide::polygonPolygon, g(ShapeDef.box(1.0, 1.0)), g(ShapeDef.box(1.0, 1.0), 1.9, 0.0))
        assertEquals(2, m.count); assertNormal(m, 1.0, 0.0)
        assertPoint(m, 0, 0.95, -1.0, 0.1); assertPoint(m, 1, 0.95, 1.0, 0.1)
    }

    @Test fun polygonPolygonVertexIntoFace() {
        val m = bothOrders(Collide::polygonPolygon, g(ShapeDef.box(1.0, 1.0)), g(ShapeDef.box(1.0, 1.0), 1.0 + sqrt(2.0) - 0.1, 0.0, PI / 4))
        assertEquals(1, m.count); assertNormal(m, 1.0, 0.0); assertEquals(0.1, -m.sep[0], 1e-9)
    }

    @Test fun polygonPolygonSeparated() {
        assertEquals(0, bothOrders(Collide::polygonPolygon, g(ShapeDef.box(1.0, 1.0)), g(ShapeDef.box(1.0, 1.0), 2.01, 0.0)).count)
    }

    // ------------------------------------------------------------ circle–segment

    @Test fun circleSegment() {
        val m = bothOrders(Collide::circleSegment, g(ShapeDef.Circle(0.5), 0.5, 0.3), g(ShapeDef.Segment(-2.0, 0.0, 2.0, 0.0)))
        assertEquals(1, m.count); assertNormal(m, 0.0, -1.0); assertPoint(m, 0, 0.5, -0.1, 0.2)
    }

    @Test fun circleSegmentPastEnd() {
        val m = bothOrders(Collide::circleSegment, g(ShapeDef.Circle(0.5), 2.3, 0.0), g(ShapeDef.Segment(-2.0, 0.0, 2.0, 0.0)))
        assertEquals(1, m.count); assertNormal(m, -1.0, 0.0); assertEquals(0.2, -m.sep[0], 1e-9)
    }

    @Test fun circleSegmentSeparated() {
        assertEquals(0, bothOrders(Collide::circleSegment, g(ShapeDef.Circle(0.5), 0.0, 0.51), g(ShapeDef.Segment(-2.0, 0.0, 2.0, 0.0))).count)
    }

    // ------------------------------------------------------------ capsule–segment

    @Test fun capsuleSegmentParallel() {
        val m = bothOrders(Collide::capsuleSegment, g(ShapeDef.Capsule(1.0, 0.5), 0.0, 0.4), g(ShapeDef.Segment(-3.0, 0.0, 3.0, 0.0)))
        assertEquals(2, m.count); assertNormal(m, 0.0, -1.0)
        assertPoint(m, 0, -1.0, -0.05, 0.1); assertPoint(m, 1, 1.0, -0.05, 0.1)
    }

    @Test fun capsuleSegmentStandingOnEnd() {
        val m = bothOrders(Collide::capsuleSegment, g(ShapeDef.Capsule(1.0, 0.5), 0.0, 1.4, PI / 2), g(ShapeDef.Segment(-3.0, 0.0, 3.0, 0.0)))
        assertEquals(1, m.count); assertNormal(m, 0.0, -1.0); assertEquals(0.1, -m.sep[0], 1e-9)
    }

    @Test fun capsuleSegmentSeparated() {
        assertEquals(0, bothOrders(Collide::capsuleSegment, g(ShapeDef.Capsule(1.0, 0.5), 0.0, 0.51), g(ShapeDef.Segment(-3.0, 0.0, 3.0, 0.0))).count)
    }

    // ------------------------------------------------------------ speculative margin

    @Test fun marginReportsNearbyShapesWithPositiveSeparation() {
        val m = Manifold()
        Collide.collide(g(ShapeDef.Circle(1.0)), g(ShapeDef.Circle(1.0), 2.01, 0.0), m, 0.02)
        assertEquals(1, m.count); assertEquals(0.01, m.sep[0], 1e-12)
        Collide.collide(g(ShapeDef.Circle(1.0)), g(ShapeDef.Circle(1.0), 2.03, 0.0), m, 0.02)
        assertEquals(0, m.count)
    }

    // ------------------------------------------------------------ properties on random pairs

    private fun randomShape(r: SplitMix64): ShapeDef = when ((r.nextLong() ushr 1) % 4) {
        0L -> ShapeDef.Circle(r.nextDouble(0.1, 1.0))
        1L -> ShapeDef.Capsule(r.nextDouble(0.1, 1.0), r.nextDouble(0.1, 0.6))
        2L -> {
            val k = 3 + ((r.nextLong() ushr 1) % 6).toInt()
            val rx = r.nextDouble(0.3, 1.2); val ry = r.nextDouble(0.3, 1.2)
            val v = DoubleArray(2 * k)
            for (i in 0 until k) {
                val a = (i + r.nextDouble(0.0, 0.4)) / k * 2 * PI
                v[2 * i] = rx * kotlin.math.cos(a); v[2 * i + 1] = ry * kotlin.math.sin(a)
            }
            ShapeDef.Polygon(v)
        }
        else -> { val h = r.nextDouble(0.2, 2.0); ShapeDef.Segment(-h, 0.0, h, 0.0) }
    }

    private fun randomPair(r: SplitMix64): Pair<ShapeDef, ShapeDef> {
        while (true) {
            val a = randomShape(r); val b = randomShape(r)
            if (a !is ShapeDef.Segment || b !is ShapeDef.Segment) return a to b
        }
    }

    @Test fun propertiesOnRandomPairs() {
        val r = SplitMix64(99L)
        val m1 = Manifold(); val m2 = Manifold()
        var contacts = 0
        repeat(5000) { iter ->
            val (sa, sb) = randomPair(r)
            val overlapping = iter % 3 == 0
            val ax = r.nextDouble(-1.0, 1.0); val ay = r.nextDouble(-1.0, 1.0)
            val a = g(sa, ax, ay, r.nextDouble(-PI, PI))
            val b = if (overlapping) g(sb, ax, ay, r.nextDouble(-PI, PI))
            else g(sb, ax + r.nextDouble(-2.5, 2.5), ay + r.nextDouble(-2.5, 2.5), r.nextDouble(-PI, PI))
            Collide.collide(a, b, m1, 0.02)
            Collide.collide(b, a, m2, 0.02)
            val ctx = "iter $iter: $sa vs $sb"
            assertEquals(m1.count, m2.count, "count in both orders, $ctx")
            assertTrue(m1.count <= 2, ctx)
            if (overlapping) {
                assertTrue(m1.count > 0, "overlapping shapes must produce a contact, $ctx")
                assertTrue((0 until m1.count).maxOf { -m1.sep[it] } >= 0.0, "penetration >= 0 when overlapping, $ctx")
            }
            if (m1.count > 0) {
                contacts++
                assertEquals(1.0, hypot(m1.nx, m1.ny), 1e-9, "unit normal, $ctx")
                assertTrue(abs(m1.nx + m2.nx) < 1e-9 && abs(m1.ny + m2.ny) < 1e-9, "normals opposite, $ctx")
                val d1 = (0 until m1.count).map { m1.sep[it] }.sorted()
                val d2 = (0 until m2.count).map { m2.sep[it] }.sorted()
                d1.zip(d2).forEach { (u, v) -> assertEquals(u, v, 1e-9, "separation in both orders, $ctx") }
                for (i in 0 until m1.count) {
                    assertTrue(m1.px[i].isFinite() && m1.py[i].isFinite() && m1.sep[i].isFinite(), ctx)
                    assertTrue(m1.sep[i] <= 0.02 + 1e-12, "no point beyond the margin, $ctx")
                }
            }
        }
        assertTrue(contacts > 1000, "the random set must exercise contacts: $contacts")
    }
}
