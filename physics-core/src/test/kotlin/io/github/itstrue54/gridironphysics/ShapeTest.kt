package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.BruteForceBroadPhase
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

/** Shape definitions, their mass properties and validation, filtering defaults, and the broad phase. */
class ShapeTest {
    private fun world() = World(WorldConfig(), FieldConfig.PRO, 1L)

    @Test fun capsuleMassProperties() {
        val w = world()
        val m = 100.0; val h = 0.5; val r = 0.3
        val b = w.body(w.createBody(BodyDef(mass = m, shape = ShapeDef.Capsule(h, r))))
        val rectArea = 4 * h * r
        val circArea = PI * r * r
        val mRect = m * rectArea / (rectArea + circArea)
        val mCirc = m - mRect
        val lc = 4 * r / (3 * PI)
        val expected = mRect * ((2 * h) * (2 * h) + (2 * r) * (2 * r)) / 12 + mCirc * (0.5 * r * r + h * h + 2 * h * lc)
        assertEquals(expected, b.inertia, 1e-9)
        assertEquals(1 / m, b.invMass, 1e-15)
        assertEquals(r, b.radius)
    }

    @Test fun boxMassProperties() {
        val w = world()
        val b = w.body(w.createBody(BodyDef(mass = 60.0, shape = ShapeDef.box(1.0, 0.5))))
        assertEquals(60.0 * (2.0 * 2.0 + 1.0 * 1.0) / 12, b.inertia, 1e-9)
    }

    @Test fun boxIsCounterClockwise() {
        val p = ShapeDef.box(2.0, 1.0)
        assertEquals(4, p.count)
        assertEquals(listOf(-2.0, -1.0, 2.0, -1.0, 2.0, 1.0, -2.0, 1.0), (0 until 8).map { p.vertex(it) })
    }

    @Test fun clockwisePolygonIsReordered() {
        val p = ShapeDef.Polygon(doubleArrayOf(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0))
        // Signed area must come out positive (CCW) after construction.
        var area2 = 0.0
        for (i in 0 until p.count) {
            val j = (i + 1) % p.count
            area2 += p.vertex(2 * i) * p.vertex(2 * j + 1) - p.vertex(2 * j) * p.vertex(2 * i + 1)
        }
        assertTrue(area2 > 0.0)
    }

    @Test fun polygonInputIsCopied() {
        val v = doubleArrayOf(0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
        val p = ShapeDef.Polygon(v)
        v[2] = 99.0
        assertEquals(1.0, p.vertex(2))
    }

    @Test fun badShapesAreRejected() {
        assertFailsWith<IllegalArgumentException> { ShapeDef.Capsule(0.0, 0.3) }
        assertFailsWith<IllegalArgumentException> { ShapeDef.Capsule(0.5, 0.0) }
        assertFailsWith<IllegalArgumentException> { ShapeDef.Polygon(doubleArrayOf(0.0, 0.0, 1.0, 0.0)) } // 2 vertices
        assertFailsWith<IllegalArgumentException> { ShapeDef.Polygon(DoubleArray(18) { if (it % 2 == 0) StrictMath.cos(it * PI / 9) else StrictMath.sin((it - 1) * PI / 9) }) } // 9
        assertFailsWith<IllegalArgumentException> { ShapeDef.Polygon(doubleArrayOf(0.0, 0.0, 2.0, 0.0, 1.0, 0.2, 2.0, 2.0, 0.0, 2.0)) } // concave
        assertFailsWith<IllegalArgumentException> { ShapeDef.Polygon(doubleArrayOf(0.0, 0.0, 1.0, 0.0, 2.0, 0.0)) } // collinear
        assertFailsWith<IllegalArgumentException> { ShapeDef.Polygon(doubleArrayOf(0.0, 0.0, 1.0, 0.0, Double.NaN, 1.0)) }
        assertFailsWith<IllegalArgumentException> { ShapeDef.Segment(1.0, 1.0, 1.0, 1.0) }
    }

    @Test fun segmentsAreStaticOnly() {
        val w = world()
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(shape = ShapeDef.Segment(0.0, 0.0, 1.0, 0.0))) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(type = BodyType.Kinematic, shape = ShapeDef.Segment(0.0, 0.0, 1.0, 0.0))) }
        w.createBody(BodyDef(type = BodyType.Static, shape = ShapeDef.Segment(0.0, 0.0, 1.0, 0.0)))
    }

    @Test fun filterBitsAre16BitAndDefaultsCollideWithEverything() {
        val d = BodyDef()
        assertEquals(Category.HOME, d.categoryBits)
        assertEquals(Category.ALL, d.maskBits)
        assertEquals(0xFFFF, Category.ALL)
        val all = listOf(Category.HOME, Category.AWAY, Category.BALL, Category.OFFICIAL, Category.STATIC, Category.LINE_SENSOR)
        assertEquals(all.size, all.toSet().size)
        all.forEach { assertEquals(1, Integer.bitCount(it)) }
        val w = world()
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(categoryBits = 0x10000)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(maskBits = -1)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(material = 99)) }
    }

    @Test fun bodyViewShowsFilterAndMaterial() {
        val w = world()
        val b = w.body(w.createBody(BodyDef(categoryBits = Category.AWAY, maskBits = Category.STATIC, material = Material.DOWNED_PLAYER)))
        assertEquals(Category.AWAY, b.categoryBits); assertEquals(Category.STATIC, b.maskBits); assertEquals(Material.DOWNED_PLAYER, b.material)
    }

    @Test fun copyFromCopiesNewFields() {
        val src = BodyDef(categoryBits = Category.BALL, maskBits = Category.STATIC, material = Material.BALL)
        val dst = BodyDef().copyFrom(src)
        assertEquals(Category.BALL, dst.categoryBits); assertEquals(Category.STATIC, dst.maskBits); assertEquals(Material.BALL, dst.material)
    }

    // ------------------------------------------------------------ broad phase

    @Test fun bruteForceBroadPhaseFindsOverlapsSortedAndOnlyActive() {
        val n = 5
        val active = booleanArrayOf(true, true, false, true, true)
        // Boxes on a line: 0 overlaps 1, 1 overlaps 2 (inactive), 3 overlaps 4, 0 overlaps 4 via wrap-around box.
        val minX = doubleArrayOf(0.0, 0.9, 1.9, 10.0, -5.0)
        val maxX = doubleArrayOf(1.0, 2.0, 3.0, 11.0, 10.5)
        val minY = DoubleArray(n) { 0.0 }
        val maxY = DoubleArray(n) { 1.0 }
        val outA = IntArray(16); val outB = IntArray(16)
        val count = BruteForceBroadPhase().findPairs(n, active, minX, minY, maxX, maxY, outA, outB)
        val pairs = (0 until count).map { outA[it] to outB[it] }
        assertEquals(listOf(0 to 1, 0 to 4, 1 to 4, 3 to 4), pairs)
    }

    @Test fun broadPhaseStopsAtOutputCapacity() {
        val n = 4
        val active = BooleanArray(n) { true }
        val lo = DoubleArray(n); val hi = DoubleArray(n) { 1.0 }
        val outA = IntArray(2); val outB = IntArray(2)
        assertEquals(2, BruteForceBroadPhase().findPairs(n, active, lo, lo, hi, hi, outA, outB))
        assertEquals(0 to 1, outA[0] to outB[0]); assertEquals(0 to 2, outA[1] to outB[1])
    }
}
