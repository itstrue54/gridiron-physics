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
        assertTrue(out[0] < out[1], "ascending")
        assertEquals(setOf(b.value, c.value), out.take(2).toSet())
        assertTrue(c.value != a.value, "a destroyed id is never handed out again")
    }

    // Final review #1: capacity counts live bodies, so one World can last a whole game.
    @Test fun capacityCountsLiveBodiesNotLifetimeCreations() {
        val w = World(WorldConfig(maxBodies = 4), FieldConfig.PRO, 1L)
        val seen = HashSet<Int>()
        repeat(10_000) {
            val id = w.createBody(BodyDef())
            assertTrue(seen.add(id.value), "id ${id.value} reused")
            w.step()
            w.destroyBody(id)
        }
        repeat(4) { w.createBody(BodyDef()) }
        assertFailsWith<IllegalStateException> { w.createBody(BodyDef()) }
    }

    @Test fun staleIdOfReusedSlotIsRejected() {
        val w = World(WorldConfig(maxBodies = 1), FieldConfig.PRO, 1L)
        val a = w.createBody(BodyDef(x = 1.0))
        w.destroyBody(a)
        val b = w.createBody(BodyDef(x = 2.0))
        assertFailsWith<IllegalArgumentException> { w.body(a) }
        assertFailsWith<IllegalArgumentException> { w.destroyBody(a) }
        assertEquals(2.0, w.body(b).x)
    }

    // Final review #3: static bodies have no velocity (the M2 solver reads it).
    @Test fun staticBodyVelocityIsZeroed() {
        val w = world()
        val s = w.body(w.createBody(BodyDef(type = BodyType.Static, vx = 3.0, vy = -1.0, angularVelocity = 2.0)))
        assertEquals(0.0, s.vx); assertEquals(0.0, s.vy); assertEquals(0.0, s.angularVelocity)
    }

    // Final review #2: a recorder must be able to keep a copy that later reuse of the def does not change.
    @Test fun bodyDefCopyFromCopiesEveryField() {
        val src = BodyDef(
            type = BodyType.Kinematic, x = 1.0, y = 2.0, angle = 3.0, vx = 4.0, vy = 5.0, angularVelocity = 6.0,
            mass = 7.0, linearDamping = 8.0, angularDamping = 9.0, shape = ShapeDef.Circle(0.31), userData = 10, bullet = true,
        )
        val dst = BodyDef().copyFrom(src)
        src.x = -1.0; src.userData = -1; src.type = BodyType.Static
        assertEquals(BodyType.Kinematic, dst.type)
        assertEquals(listOf(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0), listOf(dst.x, dst.y, dst.angle, dst.vx, dst.vy, dst.angularVelocity, dst.mass, dst.linearDamping, dst.angularDamping))
        assertEquals(0.31, (dst.shape as ShapeDef.Circle).radius)
        assertEquals(10, dst.userData); assertTrue(dst.bullet)
    }

    @Test fun recorderSeesCreateAndDestroy() {
        val log = mutableListOf<String>()
        val w = world()
        w.recorder = object : CommandRecorder {
            override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) { log += "c:${id.value}@$tick" }
            override fun onDestroyBody(tick: Long, id: BodyId) { log += "d:${id.value}@$tick" }
            override fun onSetAthlete(tick: Long, id: BodyId, params: AthleteParams) {}
            override fun onSetIntent(tick: Long, id: BodyId, intent: LocomotionIntent) {}
            override fun onSetStance(tick: Long, id: BodyId, stance: Stance) {}
            override fun onSetSurface(tick: Long, surface: SurfaceConfig) {}
            override fun onCreateGrapple(tick: Long, id: JointId, tackler: BodyId, carrier: BodyId, def: GrappleDef) {}
            override fun onCreateBlock(tick: Long, id: JointId, blocker: BodyId, defender: BodyId, def: BlockDef) {}
            override fun onCreateDistance(tick: Long, id: JointId, a: BodyId, b: BodyId, def: DistanceDef) {}
            override fun onCreateWeld(tick: Long, id: JointId, a: BodyId, b: BodyId, def: WeldDef) {}
            override fun onDestroyJoint(tick: Long, id: JointId) {}
            override fun onSetBallCarrier(tick: Long, id: BodyId?) {}
            override fun onSetAttackDirection(tick: Long, direction: Int) {}
            override fun onLaunchBall(tick: Long, from: io.github.itstrue54.gridironphysics.math.Vec3, velocity: io.github.itstrue54.gridironphysics.math.Vec3, mode: FlightMode) {}
            override fun onDeflect(tick: Long, id: BodyId, impulseX: Double, impulseY: Double, impulseZ: Double) {}
            override fun onSetWind(tick: Long, x: Double, y: Double) {}
            override fun onSetReach(tick: Long, id: BodyId, reachRadius: Double, reachHeight: Double) {}
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
        w.createBody(BodyDef()) // a live slot is free again
        assertFailsWith<IllegalStateException> { w.createBody(BodyDef()) }
        assertFailsWith<IllegalArgumentException> { w.destroyBody(a) }
        assertFailsWith<IllegalArgumentException> { w.body(a) }
        assertFailsWith<IllegalArgumentException> { w.destroyBody(BodyId(99)) }
        assertFailsWith<IllegalArgumentException> { w.body(BodyId(-1)) }
    }
}
