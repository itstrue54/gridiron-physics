package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** The spec's Testing-table rows gated by M2: Momentum conservation, Mass matters, Rest stability. */
class M2AcceptanceTest {

    /**
     * Two free circles, restitution 1, μ = 0, head-on. Production solver settings: contact softness stays on
     * (design decision 5), restitution runs as its own pass after relax.
     */
    @Test fun momentumConservation() {
        val mats = MaterialTable.DEFAULT.with(Material.PLAYER, Material.PLAYER, friction = 0.0, restitution = 1.0)
        val w = World(WorldConfig(materials = mats), FieldConfig.PRO, 1L)
        val mA = 90.0; val mB = 140.0
        val vA = 4.0; val vB = -3.0
        val a = w.createBody(BodyDef(x = 50.0, y = 20.0, vx = vA, mass = mA, shape = ShapeDef.Circle(0.35)))
        val b = w.createBody(BodyDef(x = 53.0, y = 20.0, vx = vB, mass = mB, shape = ShapeDef.Circle(0.30)))
        val p0 = mA * vA + mB * vB
        val e0 = 0.5 * mA * vA * vA + 0.5 * mB * vB * vB

        var hit = false
        repeat(120) {
            w.step()
            for (i in 0 until w.contactCount) if (w.contact(i).normalImpulse(0) > 0.0) hit = true
        }
        assertTrue(hit, "the circles must collide")

        val ba = w.body(a); val vA1 = ba.vx; val vyA = ba.vy; val wA = ba.angularVelocity
        val bb = w.body(b); val vB1 = bb.vx; val vyB = bb.vy; val wB = bb.angularVelocity
        val p1 = mA * vA1 + mB * vB1
        val e1 = 0.5 * mA * (vA1 * vA1 + vyA * vyA) + 0.5 * mB * (vB1 * vB1 + vyB * vyB)
        assertTrue(kotlin.math.abs(p1 - p0) < 1e-9, "momentum error ${p1 - p0}")
        assertTrue(kotlin.math.abs(mA * vyA + mB * vyB) < 1e-9, "no sideways momentum")
        assertTrue(kotlin.math.abs(e1 - e0) / e0 < 1e-6, "relative KE error ${(e1 - e0) / e0}")
        assertEquals(0.0, wA, 1e-12); assertEquals(0.0, wB, 1e-12)
        // 1D elastic collision, analytic.
        assertEquals(((mA - mB) * vA + 2 * mB * vB) / (mA + mB), vA1, 1e-9)
        assertEquals(((mB - mA) * vB + 2 * mA * vA) / (mA + mB), vB1, 1e-9)
    }

    /** 140 kg vs 90 kg, equal speed, head-on, default materials: the heavier body keeps moving forward. */
    @Test fun massMatters() {
        val w = World(WorldConfig(), FieldConfig.PRO, 1L)
        val heavy = w.createBody(BodyDef(x = 50.0, y = 20.0, vx = 7.0, mass = 140.0, shape = ShapeDef.Circle(0.36)))
        val light = w.createBody(BodyDef(x = 54.0, y = 20.0, vx = -7.0, mass = 90.0, shape = ShapeDef.Circle(0.32)))
        repeat(60) { w.step() }
        assertTrue(w.body(heavy).vx > 0.0, "heavy keeps moving forward: ${w.body(heavy).vx}")
        assertTrue(w.body(light).vx > 0.0, "light is driven backward: ${w.body(light).vx}")
        assertTrue(w.body(heavy).x < w.body(light).x, "no tunneling")
    }

    /** 22 standing players (linemen shoulder to shoulder and nose to nose), zero intent, 10 s. */
    @Test fun restStability() {
        val w = World(WorldConfig(), FieldConfig.PRO, 1L)
        val f = FieldConfig.PRO
        val r = 0.34
        val cx = f.midfieldX
        val cy = f.widthM / 2
        val ids = ArrayList<BodyId>()
        fun p(x: Double, y: Double, mass: Double, tag: Int) {
            ids += w.createBody(BodyDef(x = x, y = y, mass = mass, linearDamping = 0.5, angularDamping = 0.5, shape = ShapeDef.Circle(r), userData = tag))
        }
        // Offensive line, touching each other, and the defensive line touching them across the ball.
        for (k in -2..2) p(cx - r, cy + k * 2 * r, 140.0 + k, 0)
        for (k in -1..2) p(cx + r, cy + k * 2 * r, 130.0 + k, 1)
        // Skill players and backers, apart.
        val off = listOf(-5.0 to 0.0, -7.0 to 0.0, -1.0 to 8.0, -1.0 to -12.0, -2.0 to 14.0, -2.0 to -14.0)
        val def = listOf(5.0 to 0.0, 5.0 to 4.0, 5.0 to -4.0, 3.0 to 14.0, 3.0 to -14.0, 12.0 to 6.0, 12.0 to -6.0)
        off.forEach { (dx, dy) -> p(cx + dx, cy + dy, 95.0, 0) }
        def.forEach { (dx, dy) -> p(cx + dx, cy + dy, 100.0, 1) }
        assertEquals(22, ids.size)

        val x0 = ids.map { w.body(it).x }
        val y0 = ids.map { w.body(it).y }
        var maxDrift = 0.0
        for (t in 1..600) {
            w.step()
            ids.forEachIndexed { i, id ->
                val b = w.body(id)
                maxDrift = maxOf(maxDrift, kotlin.math.hypot(b.x - x0[i], b.y - y0[i]))
            }
            if (t >= 60) ids.forEach { assertFalse(w.body(it).awake, "body ${it.value} still awake at tick $t") }
        }
        assertTrue(maxDrift < 1e-3, "max drift $maxDrift m")
    }
}
