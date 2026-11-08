package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class WorldBodiesTest {
    private fun world() = World(WorldConfig(), FieldConfig.PRO, seed = 1L)

    @Test fun constantVelocityBodyMovesVdtPerTick() {
        val w = world()
        val id = w.createBody(BodyDef(x = 10.0, y = 20.0, vx = 6.0, vy = -3.0))
        repeat(60) { w.step() }
        val b = w.body(id)
        assertEquals(16.0, b.x, 1e-9)
        assertEquals(17.0, b.y, 1e-9)
        assertEquals(60L, w.tick)
    }

    @Test fun angularVelocityIntegratesAngle() {
        val w = world()
        val id = w.createBody(BodyDef(angularVelocity = 1.5))
        repeat(60) { w.step() }
        assertEquals(1.5, w.body(id).angle, 1e-9)
    }

    @Test fun linearDampingFollowsBox2dFormula() {
        val w = world()
        val id = w.createBody(BodyDef(vx = 10.0, linearDamping = 2.0, angularVelocity = 4.0, angularDamping = 1.0))
        w.step()
        assertEquals(10.0 / (1.0 + WorldConfig.DT * 2.0), w.body(id).vx, 1e-12)
        assertEquals(4.0 / (1.0 + WorldConfig.DT * 1.0), w.body(id).angularVelocity, 1e-12)
    }

    @Test fun staticNeverMovesKinematicIgnoresDamping() {
        val w = world()
        val s = w.createBody(BodyDef(type = BodyType.Static, x = 5.0, vx = 3.0))
        val k = w.createBody(BodyDef(type = BodyType.Kinematic, vx = 3.0, linearDamping = 5.0))
        repeat(60) { w.step() }
        assertEquals(5.0, w.body(s).x)
        assertEquals(3.0, w.body(k).x, 1e-9)
        assertEquals(3.0, w.body(k).vx)
    }

    @Test fun prevPositionIsPositionBeforeLastStep() {
        val w = world()
        val id = w.createBody(BodyDef(vx = 6.0, vy = 3.0))
        w.step(); w.step()
        assertEquals(0.1, w.body(id).prevX, 1e-12)
        assertEquals(0.05, w.body(id).prevY, 1e-12)
        assertEquals(0.2, w.body(id).x, 1e-12)
    }

    @Test fun newBodyPrevEqualsPosition() {
        val w = world()
        val b = w.body(w.createBody(BodyDef(x = 3.0, y = 4.0)))
        assertEquals(3.0, b.prevX); assertEquals(4.0, b.prevY)
    }

    @Test fun circleMassProperties() {
        val w = world()
        val b = w.body(w.createBody(BodyDef(mass = 100.0, shape = ShapeDef.Circle(0.3))))
        assertEquals(0.01, b.invMass, 1e-15)
        assertEquals(0.5 * 100.0 * 0.09, b.inertia, 1e-12)
        assertEquals(0.3, b.radius)
        val s = w.body(w.createBody(BodyDef(type = BodyType.Static)))
        assertEquals(0.0, s.invMass)
        val k = w.body(w.createBody(BodyDef(type = BodyType.Kinematic)))
        assertEquals(0.0, k.invMass)
    }

    @Test fun bodyViewCarriesDefFields() {
        val w = world()
        val b = w.body(w.createBody(BodyDef(userData = 42, bullet = true, linearDamping = 0.7)))
        assertEquals(42, b.userData); assertTrue(b.bullet); assertEquals(0.7, b.linearDamping); assertTrue(b.awake)
    }

    @Test fun bodyIdsAscendingAndNeverReused() {
        val w = world()
        val a = w.createBody(BodyDef())
        val b = w.createBody(BodyDef())
        w.destroyBody(a)
        val c = w.createBody(BodyDef())
        val out = IntArray(8)
        assertEquals(2, w.bodyIds(out))
        assertEquals(listOf(b.value, c.value), out.take(2))
        assertTrue(c.value > b.value)
    }

    @Test fun recorderSeesCreateAndDestroy() {
        val log = mutableListOf<String>()
        val w = world()
        w.recorder = object : CommandRecorder {
            override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) { log += "c:${id.value}@$tick" }
            override fun onDestroyBody(tick: Long, id: BodyId) { log += "d:${id.value}@$tick" }
        }
        val id = w.createBody(BodyDef())
        w.step()
        w.destroyBody(id)
        assertEquals(listOf("c:0@0", "d:0@1"), log)
    }

    // Review Focus 4
    @Test fun rejectsBadBodyData() {
        val w = world()
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(x = Double.NaN)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(vy = Double.POSITIVE_INFINITY)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(angle = Double.NaN)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(mass = 0.0)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(mass = Double.NaN)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(linearDamping = -1.0)) }
        assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(shape = ShapeDef.Circle(0.0))) }
        // Static bodies need no mass.
        w.createBody(BodyDef(type = BodyType.Static, mass = 0.0))
    }

    // Review Focus 5
    @Test fun idMisuseFailsClearly() {
        val w = World(WorldConfig(maxBodies = 2), FieldConfig.PRO, 1L)
        val a = w.createBody(BodyDef())
        w.createBody(BodyDef())
        assertFailsWith<IllegalStateException> { w.createBody(BodyDef()) }
        w.destroyBody(a)
        assertFailsWith<IllegalStateException> { w.createBody(BodyDef()) } // ids are never reused
        assertFailsWith<IllegalArgumentException> { w.destroyBody(a) }
        assertFailsWith<IllegalArgumentException> { w.body(a) }
        assertFailsWith<IllegalArgumentException> { w.destroyBody(BodyId(99)) }
        assertFailsWith<IllegalArgumentException> { w.body(BodyId(-1)) }
    }
}
