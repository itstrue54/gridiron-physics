package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
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

    @Test fun registryListsDrift() {
        assertTrue(Scenarios.drift in Scenarios.all)
        assertEquals(Scenarios.all.size, Scenarios.all.map { it.name }.toSet().size, "scenario names are unique")
    }
}
