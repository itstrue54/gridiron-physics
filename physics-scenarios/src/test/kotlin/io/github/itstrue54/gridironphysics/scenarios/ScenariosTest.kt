package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.math.DMath
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ScenariosTest {
    private fun ids(w: World): List<BodyId> {
        val out = IntArray(64)
        val n = w.bodyIds(out)
        return (0 until n).map { BodyId(out[it]) }
    }

    @Test fun driftHas22PlayersOneOfficialOneProp() {
        val w = Scenarios.drift.build()
        val ids = ids(w)
        assertEquals(24, ids.size)
        assertEquals(11, ids.count { w.body(it).userData == Tags.HOME })
        assertEquals(11, ids.count { w.body(it).userData == Tags.AWAY })
        assertEquals(1, ids.count { w.body(it).userData == Tags.OFFICIAL && w.body(it).type == BodyType.Kinematic })
        assertEquals(1, ids.count { w.body(it).userData == Tags.PROP && w.body(it).type == BodyType.Static })
    }

    @Test fun driftPlayersUseSpecRanges() {
        val w = Scenarios.drift.build()
        ids(w).filter { w.body(it).userData <= Tags.AWAY }.forEach {
            val b = w.body(it)
            assertTrue(b.mass in 80.0..160.0, "mass ${b.mass}")
            assertTrue(b.radius in 0.30..0.38, "radius ${b.radius}")
        }
    }

    @Test fun driftSlowsDownAndStaysFinite() {
        val w = Scenarios.drift.build()
        val ids = ids(w)
        fun maxPlayerSpeed() = ids.filter { w.body(it).userData <= Tags.AWAY }
            .maxOf { val b = w.body(it); DMath.hypot2(b.vx, b.vy) }
        val v0 = maxPlayerSpeed()
        assertTrue(v0 > 1.0, "players should start moving")
        repeat(600) { w.step() }
        assertTrue(maxPlayerSpeed() < v0 * 0.05)
        ids.forEach { val b = w.body(it); assertTrue(b.x.isFinite() && b.y.isFinite()) }
    }

    @Test fun buildIsDeterministic() {
        assertEquals(Scenarios.drift.build().stateHash(), Scenarios.drift.build().stateHash())
    }

    // ------------------------------------------------------------ M2: head-on tackle (collision only)

    @Test fun headOnTackleIsA90KgBackVsA115KgLinebacker() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        assertEquals(2, ids.size)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        assertEquals(90.0, w.body(back).mass); assertEquals(115.0, w.body(lb).mass)
        assertTrue(w.body(back).vx > 8.0 && w.body(lb).vx < -6.0, "both at full speed, head-on")
    }

    @Test fun headOnTackleCollidesWithoutTunnelingAndConservesMomentum() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        fun px() = w.body(back).mass * w.body(back).vx + w.body(lb).mass * w.body(lb).vx
        var hit = false
        var maxDepth = 0.0
        var pBefore = px()
        var pAfter = 0.0
        repeat(180) {
            val p = px()
            w.step()
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                if (c.normalImpulse(0) > 0.0 && !hit) { hit = true; pBefore = p; pAfter = px() }
                for (k in 0 until c.pointCount) maxDepth = maxOf(maxDepth, -c.separation(k))
            }
        }
        assertTrue(hit, "they must collide")
        assertTrue(w.body(back).x < w.body(lb).x, "no tunneling")
        assertTrue(maxDepth < 0.01, "max penetration $maxDepth")
        // Damping acts during the impact tick, so allow a small tolerance.
        assertEquals(pBefore, pAfter, kotlin.math.abs(pBefore) * 0.02 + 5.0)
    }

    // ------------------------------------------------------------ M2: contact lab (shapes, walls, sleeping)

    @Test fun contactLabUsesEveryShapeKind() {
        val w = Scenarios.contactLab.build()
        val shapes = ids(w).map { w.body(it).shape::class }.toSet()
        assertTrue(ShapeDef.Circle::class in shapes)
        assertTrue(ShapeDef.Capsule::class in shapes)
        assertTrue(ShapeDef.Polygon::class in shapes)
        assertTrue(ShapeDef.Segment::class in shapes)
    }

    @Test fun contactLabKeepsEveryoneInsideTheWorldAndSettles() {
        val w = Scenarios.contactLab.build()
        repeat(600) { w.step() }
        val f = w.field
        ids(w).forEach {
            val b = w.body(it)
            assertTrue(b.x > -f.runoffM && b.x < f.lengthM + f.runoffM && b.y > -f.runoffM && b.y < f.widthM + f.runoffM, "body ${it.value} left the world")
        }
        assertTrue(ids(w).filter { w.body(it).type == BodyType.Dynamic }.all { !w.body(it).awake }, "everything has come to rest after 10 s")
    }

    @Test fun registryListsDrift() {
        assertTrue(Scenarios.drift in Scenarios.all)
        assertTrue(Scenarios.headOnTackle in Scenarios.all)
        assertTrue(Scenarios.contactLab in Scenarios.all)
        assertEquals(Scenarios.all.size, Scenarios.all.map { it.name }.toSet().size, "scenario names are unique")
    }
}
