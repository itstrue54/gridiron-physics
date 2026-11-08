package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals

class StateHashTest {
    private fun world(cfg: WorldConfig = WorldConfig()) = World(cfg, FieldConfig.PRO, seed = 1L)

    @Test fun emptyWorldHashIsStable() {
        assertEquals(world().stateHash(), world().stateHash())
    }

    @Test fun hashChangesWithAnyBodyField() {
        val base = BodyDef(x = 1.0, y = 2.0, angle = 0.3, vx = 4.0, vy = 5.0, angularVelocity = 0.6)
        fun hashOf(def: BodyDef): Long = world().also { it.createBody(def) }.stateHash()
        fun copy() = BodyDef(x = base.x, y = base.y, angle = base.angle, vx = base.vx, vy = base.vy, angularVelocity = base.angularVelocity)
        val h = hashOf(copy())
        assertEquals(h, hashOf(copy()))
        assertNotEquals(h, hashOf(copy().apply { x = Math.nextUp(x) }), "x")
        assertNotEquals(h, hashOf(copy().apply { y = Math.nextUp(y) }), "y")
        assertNotEquals(h, hashOf(copy().apply { angle = Math.nextUp(angle) }), "angle")
        assertNotEquals(h, hashOf(copy().apply { vx = Math.nextUp(vx) }), "vx")
        assertNotEquals(h, hashOf(copy().apply { vy = Math.nextUp(vy) }), "vy")
        assertNotEquals(h, hashOf(copy().apply { angularVelocity = Math.nextUp(angularVelocity) }), "angularVelocity")
    }

    @Test fun hashChangesWithBodyCount() {
        val w1 = world().also { it.createBody(BodyDef()) }
        val w2 = world().also { it.createBody(BodyDef()); it.createBody(BodyDef(type = BodyType.Static)) }
        assertNotEquals(w1.stateHash(), w2.stateHash())
    }

    @Test fun hashChangesWithTick() {
        val w = world()
        val h0 = w.stateHash()
        w.step()
        assertNotEquals(h0, w.stateHash())
    }

    @Test fun hashEveryTickMatchesStateHash() {
        val w = world(WorldConfig(hashEveryTick = true))
        w.createBody(BodyDef(vx = 1.0))
        w.step()
        assertEquals(w.stateHash(), w.lastTickHash)
    }
}
