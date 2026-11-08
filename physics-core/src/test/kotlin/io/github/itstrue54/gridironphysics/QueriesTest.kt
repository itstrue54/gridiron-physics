package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Vec2
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class QueriesTest {
    private fun world() = World(WorldConfig(), FieldConfig.PRO, 1L)
    private val all = QueryFilter()
    private val hit = RayHit()

    private fun static(w: World, x: Double, y: Double, shape: ShapeDef, angle: Double = 0.0, category: Int = Category.STATIC): BodyId =
        w.createBody(BodyDef(type = BodyType.Static, x = x, y = y, angle = angle, shape = shape, categoryBits = category))

    private fun ray(w: World, x1: Double, y1: Double, x2: Double, y2: Double, filter: QueryFilter = all) =
        w.raycast(Vec2(x1, y1), Vec2(x2, y2), filter, hit)

    private fun cast(w: World, x1: Double, y1: Double, x2: Double, y2: Double, r: Double, filter: QueryFilter = all) =
        w.circleCast(Vec2(x1, y1), Vec2(x2, y2), r, filter, hit)

    private fun assertHit(id: BodyId, fraction: Double, px: Double, py: Double, nx: Double, ny: Double) {
        assertEquals(id, hit.body, "$hit")
        assertEquals(fraction, hit.fraction, 1e-9, "fraction $hit")
        assertEquals(px, hit.pointX, 1e-9, "pointX $hit"); assertEquals(py, hit.pointY, 1e-9, "pointY $hit")
        assertEquals(nx, hit.normalX, 1e-9, "normalX $hit"); assertEquals(ny, hit.normalY, 1e-9, "normalY $hit")
    }

    // ------------------------------------------------------------ raycast, each shape from both sides

    @Test fun raycastCircleFromBothSides() {
        val w = world()
        val c = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0)); assertHit(c, 0.45, 49.0, 20.0, -1.0, 0.0)
        assertTrue(ray(w, 60.0, 20.0, 40.0, 20.0)); assertHit(c, 0.45, 51.0, 20.0, 1.0, 0.0)
    }

    @Test fun raycastMissesAndStopsShort() {
        val w = world()
        static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertFalse(ray(w, 40.0, 22.0, 60.0, 22.0), "passes above")
        assertFalse(ray(w, 40.0, 20.0, 48.0, 20.0), "ends before the circle")
        assertFalse(ray(w, 48.0, 20.0, 40.0, 20.0), "points away")
    }

    @Test fun raycastCapsuleSideAndCaps() {
        val w = world()
        val c = static(w, 50.0, 20.0, ShapeDef.Capsule(1.0, 0.25))
        assertTrue(ray(w, 50.0, 10.0, 50.0, 30.0)); assertHit(c, 9.75 / 20, 50.0, 19.75, 0.0, -1.0)
        assertTrue(ray(w, 50.0, 30.0, 50.0, 10.0)); assertHit(c, 9.75 / 20, 50.0, 20.25, 0.0, 1.0)
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0)); assertHit(c, 8.75 / 20, 48.75, 20.0, -1.0, 0.0)
        val dx = DMath.sqrt(0.25 * 0.25 - 0.1 * 0.1)
        assertTrue(ray(w, 60.0, 20.1, 40.0, 20.1)); assertHit(c, (60.0 - 51.0 - dx) / 20, 51.0 + dx, 20.1, dx / 0.25, 0.1 / 0.25)
    }

    @Test fun raycastPolygonFacesAndRotation() {
        val w = world()
        val box = static(w, 50.0, 20.0, ShapeDef.box(1.0, 0.5))
        assertTrue(ray(w, 40.0, 20.2, 60.0, 20.2)); assertHit(box, 0.45, 49.0, 20.2, -1.0, 0.0)
        assertTrue(ray(w, 60.0, 20.2, 40.0, 20.2)); assertHit(box, 0.45, 51.0, 20.2, 1.0, 0.0)
        assertTrue(ray(w, 50.5, 10.0, 50.5, 30.0)); assertHit(box, 9.5 / 20, 50.5, 19.5, 0.0, -1.0)
        val w2 = world()
        val turned = static(w2, 50.0, 20.0, ShapeDef.box(1.0, 0.5), angle = DMath.PI / 2)
        assertTrue(ray(w2, 40.0, 20.2, 60.0, 20.2))
        assertEquals(turned, hit.body); assertEquals(49.5, hit.pointX, 1e-9)
    }

    @Test fun raycastSegmentFromBothSides() {
        val w = world()
        val s = static(w, 50.0, 20.0, ShapeDef.Segment(0.0, -5.0, 0.0, 5.0))
        assertTrue(ray(w, 40.0, 21.0, 60.0, 21.0)); assertHit(s, 0.5, 50.0, 21.0, -1.0, 0.0)
        assertTrue(ray(w, 60.0, 21.0, 40.0, 21.0)); assertHit(s, 0.5, 50.0, 21.0, 1.0, 0.0)
        assertFalse(ray(w, 40.0, 26.0, 60.0, 26.0), "beyond the segment's end")
    }

    @Test fun raycastStartingInsideHitsAtZero() {
        val w = world()
        val c = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertTrue(ray(w, 50.2, 20.0, 60.0, 20.0)); assertHit(c, 0.0, 50.2, 20.0, -1.0, 0.0)
    }

    @Test fun raycastReportsTheClosestBodyLowerIdOnTies() {
        for (nearFirst in listOf(true, false)) {
            val w = world()
            val near: BodyId; val far: BodyId
            if (nearFirst) { near = static(w, 50.0, 20.0, ShapeDef.Circle(1.0)); far = static(w, 55.0, 20.0, ShapeDef.Circle(1.0)) }
            else { far = static(w, 55.0, 20.0, ShapeDef.Circle(1.0)); near = static(w, 50.0, 20.0, ShapeDef.Circle(1.0)) }
            assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0)); assertEquals(near, hit.body, "nearFirst=$nearFirst")
            assertTrue(ray(w, 60.0, 20.0, 40.0, 20.0)); assertEquals(far, hit.body, "nearFirst=$nearFirst")
        }
        val w = world()
        val first = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0)); assertEquals(first, hit.body)
    }

    @Test fun filterSkipsMaskedIgnoredAndSensorBodies() {
        val w = world()
        val sensor = static(w, 45.0, 20.0, ShapeDef.Circle(1.0, sensor = true))
        val home = static(w, 50.0, 20.0, ShapeDef.Circle(1.0), category = Category.HOME)
        val away = static(w, 55.0, 20.0, ShapeDef.Circle(1.0), category = Category.AWAY)
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0)); assertEquals(home, hit.body, "sensors are skipped by default")
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0, QueryFilter(includeSensors = true))); assertEquals(sensor, hit.body)
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0, QueryFilter(maskBits = Category.AWAY))); assertEquals(away, hit.body)
        assertTrue(ray(w, 40.0, 20.0, 60.0, 20.0, QueryFilter().ignore(home))); assertEquals(away, hit.body)
        assertFalse(ray(w, 40.0, 20.0, 60.0, 20.0, QueryFilter(maskBits = Category.BALL)))
    }

    @Test fun zeroLengthCastsAreRejected() {
        val w = world()
        assertFailsWith<IllegalArgumentException> { ray(w, 1.0, 1.0, 1.0, 1.0) }
        assertFailsWith<IllegalArgumentException> { cast(w, 1.0, 1.0, 1.0, 1.0, 0.3) }
        assertFailsWith<IllegalArgumentException> { cast(w, 1.0, 1.0, 2.0, 1.0, -0.1) }
    }

    // ------------------------------------------------------------ circle cast

    @Test fun circleCastStopsAtFirstTouchFromBothSides() {
        val w = world()
        val c = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertTrue(cast(w, 40.0, 20.0, 60.0, 20.0, 0.5)); assertHit(c, 0.425, 48.5, 20.0, -1.0, 0.0)
        assertTrue(cast(w, 60.0, 20.0, 40.0, 20.0, 0.5)); assertHit(c, 0.425, 51.5, 20.0, 1.0, 0.0)
    }

    @Test fun circleCastAgainstPolygonFaceAndRoundedCorner() {
        val w = world()
        val box = static(w, 50.0, 20.0, ShapeDef.box(1.0, 0.5))
        assertTrue(cast(w, 40.0, 20.2, 60.0, 20.2, 0.5)); assertHit(box, 0.425, 48.5, 20.2, -1.0, 0.0)
        // Passing 0.4 m above the top face: it touches the corner (49, 20.5) when 0.5 m from it.
        assertTrue(cast(w, 40.0, 20.9, 60.0, 20.9, 0.5)); assertHit(box, 8.7 / 20, 48.7, 20.9, -0.6, 0.8)
        assertFalse(cast(w, 40.0, 21.1, 60.0, 21.1, 0.5), "clears the box")
    }

    @Test fun circleCastAgainstCapsuleAndSegment() {
        val w = world()
        val cap = static(w, 50.0, 20.0, ShapeDef.Capsule(1.0, 0.25))
        assertTrue(cast(w, 50.0, 10.0, 50.0, 30.0, 0.5)); assertHit(cap, 9.25 / 20, 50.0, 19.25, 0.0, -1.0)
        val w2 = world()
        val seg = static(w2, 50.0, 20.0, ShapeDef.Segment(0.0, -5.0, 0.0, 5.0))
        assertTrue(cast(w2, 60.0, 21.0, 40.0, 21.0, 0.5)); assertHit(seg, 9.5 / 20, 50.5, 21.0, 1.0, 0.0)
        // Around the segment's end: the end point acts like a circle of the cast radius.
        assertTrue(cast(w2, 40.0, 25.3, 60.0, 25.3, 0.5)); assertHit(seg, 9.6 / 20, 49.6, 25.3, -0.8, 0.6)
    }

    @Test fun circleCastStartingInOverlapHitsAtZero() {
        val w = world()
        val c = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        assertTrue(cast(w, 48.8, 20.0, 40.0, 20.0, 0.5)); assertHit(c, 0.0, 48.8, 20.0, 1.0, 0.0)
    }

    // ------------------------------------------------------------ area and nearest

    @Test fun queryAabbListsOverlappingBodiesInIdOrder() {
        val w = world()
        val a = static(w, 50.0, 20.0, ShapeDef.Circle(1.0))
        static(w, 60.0, 20.0, ShapeDef.Circle(1.0))
        val c = static(w, 45.0, 21.0, ShapeDef.box(0.5, 0.5))
        val s = static(w, 47.0, 20.0, ShapeDef.Circle(0.5, sensor = true))
        val out = IntArray(8)
        assertEquals(2, w.queryAabb(44.0, 19.0, 49.1, 22.0, all, out), "touches the circle's edge at 49")
        assertEquals(listOf(a.value, c.value), out.take(2))
        assertEquals(3, w.queryAabb(44.0, 19.0, 49.1, 22.0, QueryFilter(includeSensors = true), out))
        assertEquals(listOf(a.value, c.value, s.value), out.take(3))
        val small = IntArray(1)
        assertEquals(1, w.queryAabb(0.0, 0.0, 100.0, 40.0, all, small), "stops when out is full")
        assertEquals(a.value, small[0])
        assertEquals(0, w.queryAabb(70.0, 0.0, 80.0, 40.0, all, out))
        assertFailsWith<IllegalArgumentException> { w.queryAabb(10.0, 0.0, 5.0, 1.0, all, out) }
    }

    @Test fun nearestBodiesAreSortedByDistanceLowerIdOnTies() {
        val w = world()
        val far = static(w, 53.0, 20.0, ShapeDef.Circle(0.3))
        val near = static(w, 51.0, 20.0, ShapeDef.Circle(0.3))
        val mid = static(w, 50.0, 22.0, ShapeDef.Circle(0.3))
        val tie = static(w, 50.0, 18.0, ShapeDef.Circle(0.3)) // same distance as mid, higher id
        val out = IntArray(8)
        val p = Vec2(50.0, 20.0)
        assertEquals(4, w.nearestBodies(p, 8, all, out))
        assertEquals(listOf(near.value, mid.value, tie.value, far.value), out.take(4))
        assertEquals(2, w.nearestBodies(p, 2, all, out))
        assertEquals(listOf(near.value, mid.value), out.take(2))
        assertEquals(2, w.nearestBodies(p, 2, QueryFilter().ignore(near), out))
        assertEquals(listOf(mid.value, tie.value), out.take(2))
        assertEquals(0, w.nearestBodies(p, 0, all, out))
        assertFailsWith<IllegalArgumentException> { w.nearestBodies(p, 9, all, out) }
    }

    // ------------------------------------------------------------ queries see the world as of the last step

    @Test fun queriesSeeEndOfStepPositionsAndSkipDestroyedBodies() {
        val w = world()
        val p = w.createBody(BodyDef(x = 40.0, y = 20.0, vx = 6.0, shape = ShapeDef.Circle(0.34)))
        repeat(30) { w.step() }
        val x = w.body(p).x
        assertEquals(43.0, x, 1e-9)
        assertTrue(ray(w, 30.0, 20.0, 50.0, 20.0)); assertEquals(p, hit.body)
        assertEquals(x - 0.34, hit.pointX, 1e-9)
        w.destroyBody(p)
        assertFalse(ray(w, 30.0, 20.0, 50.0, 20.0))
        assertEquals(0, w.nearestBodies(Vec2(40.0, 20.0), 1, all, IntArray(1)))
    }
}
