package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.SplitMix64
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNotEquals
import kotlin.test.assertSame
import kotlin.test.assertTrue

class LocomotionTest {
    private val dt = WorldConfig.DT

    /** Traction high enough that it never clamps, to see the controller's own limits. */
    private val grippy = AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 10.0)

    private fun world() = World(WorldConfig(), FieldConfig.PRO, 1L)
    private fun speed(w: World, id: BodyId) = w.body(id).let { DMath.hypot2(it.vx, it.vy) }
    private fun heading(w: World, id: BodyId) = w.body(id).let { DMath.atan2(it.vy, it.vx) }

    private fun runner(w: World, x: Double = 30.0, y: Double = 24.0, vx: Double = 0.0, vy: Double = 0.0, ap: AthleteParams = grippy): BodyId {
        val id = w.createBody(BodyDef(x = x, y = y, vx = vx, vy = vy, angle = DMath.atan2(vy, vx), mass = 95.0, shape = ShapeDef.Circle(0.33)))
        w.setAthlete(id, ap)
        return id
    }

    // ------------------------------------------------------------ speed control

    @Test fun decelerationUsesMaxDecelWithoutTaper() {
        val w = world()
        val p = runner(w, vx = 9.0)
        w.setIntent(p, LocomotionIntent.velocity(0.0, 0.0))
        var ticks = 0
        while (speed(w, p) > 0.0 && ticks < 200) {
            val v0 = speed(w, p)
            w.step(); ticks++
            val dv = v0 - speed(w, p)
            if (speed(w, p) > 0.0) assertEquals(grippy.maxDecel * dt, dv, 1e-9, "constant deceleration")
        }
        assertEquals(9.0 / grippy.maxDecel, ticks * dt, dt, "stops in maxSpeed/maxDecel")
        repeat(30) { w.step() }
        assertEquals(0.0, speed(w, p), "stays stopped, never backs up")
    }

    @Test fun effortCapsSpeed() {
        val w = world()
        val p = runner(w, x = 10.0)
        w.setIntent(p, LocomotionIntent.velocity(20.0, 0.0, effort = 0.5))
        repeat(600) {
            w.step()
            assertTrue(speed(w, p) <= 4.5 * (1 + 1e-9))
        }
        assertEquals(4.5, speed(w, p), 0.01 * 4.5)
    }

    @Test fun targetVelocityBelowCapIsHeld() {
        val w = world()
        val p = runner(w, x = 10.0, y = 5.0)
        w.setIntent(p, LocomotionIntent.velocity(3.0, 4.0)) // 5 m/s, under the 9 m/s cap
        repeat(300) { w.step() }
        assertEquals(3.0, w.body(p).vx, 0.02); assertEquals(4.0, w.body(p).vy, 0.02)
    }

    @Test fun pointTargetArrivesAndStops() {
        val w = world()
        val p = runner(w)
        w.setIntent(p, LocomotionIntent.point(40.0, 29.0))
        repeat(480) { w.step() }
        val b = w.body(p)
        assertEquals(40.0, b.x, 0.1); assertEquals(29.0, b.y, 0.1)
        assertTrue(speed(w, p) < 0.05, "stopped at the point, speed ${speed(w, p)}")
    }

    @Test fun idleIntentAppliesNoForce() {
        val w = world()
        val p = runner(w, vx = 4.0)
        w.setIntent(p, LocomotionIntent()) // Idle
        repeat(10) { w.step() }
        assertEquals(4.0, w.body(p).vx, 1e-12)
    }

    // ------------------------------------------------------------ turning

    @Test fun turnRateFallsAsSpeedRises() {
        for (s in listOf(9.0, 4.5, 2.0)) {
            val w = world()
            val p = runner(w, vx = s)
            w.setIntent(p, LocomotionIntent.velocity(s, 0.0))
            w.step()
            w.setIntent(p, LocomotionIntent.velocity(0.0, s)) // 90° cut
            val h0 = heading(w, p)
            w.step()
            val expected = grippy.turnRateAtRest + (grippy.turnRateAtMax - grippy.turnRateAtRest) * (s / grippy.maxSpeed)
            assertEquals(expected * dt, heading(w, p) - h0, 1e-9, "turn per tick at $s m/s")
        }
    }

    @Test fun headingNeverTurnsFasterThanTheTurnRate() {
        val w = world()
        val p = runner(w, vx = 9.0)
        w.setIntent(p, LocomotionIntent.velocity(-9.0, 1.0)) // nearly a reversal
        repeat(240) {
            val s = speed(w, p)
            val h0 = heading(w, p)
            w.step()
            if (s > 1.0 && speed(w, p) > 1.0) {
                var d = heading(w, p) - h0
                if (d > DMath.PI) d -= 2 * DMath.PI
                if (d < -DMath.PI) d += 2 * DMath.PI
                val rate = grippy.turnRateAtRest + (grippy.turnRateAtMax - grippy.turnRateAtRest) * (s / grippy.maxSpeed)
                assertTrue(DMath.abs(d) <= rate * dt + 1e-9, "turned ${d / dt} rad/s at $s m/s, limit $rate")
            }
        }
        assertTrue(w.body(p).vx < -8.0, "the reversal completes: ${w.body(p).vx}")
    }

    @Test fun playerFacesWhereHeRuns() {
        val w = world()
        val p = runner(w)
        w.setIntent(p, LocomotionIntent.velocity(0.0, -6.0))
        repeat(120) { w.step() }
        assertEquals(-DMath.PI / 2, w.body(p).angle, 0.02)
        assertEquals(0.0, w.body(p).angularVelocity, 0.05)
    }

    @Test fun angleTurnsTowardTheTargetFromRestAtTheRestTurnRate() {
        val w = world()
        val p = runner(w)
        w.setIntent(p, LocomotionIntent.point(30.0, 34.0, effort = 1.0)) // straight left of a player facing +x
        w.step()
        assertEquals(grippy.turnRateAtRest * dt, w.body(p).angle, 1e-6)
    }

    // ------------------------------------------------------------ traction and slip

    private fun slips(w: World) = w.drain().filter { it.type == EventType.Slip }

    @Test fun slipEventNamesTheBodyOnlyWhileItSlips() {
        val ap = grippy.copy(traction = 0.9)
        val w = world()
        val a = runner(w, vx = 9.0, ap = ap)
        val b = runner(w, y = 40.0, vx = 9.0, ap = ap)
        w.setIntent(a, LocomotionIntent.velocity(9.0, 0.0))
        w.setIntent(b, LocomotionIntent.velocity(-9.0, 0.0)) // reversal: hard braking beyond traction
        w.step()
        val s = slips(w)
        assertEquals(1, s.size)
        assertEquals(b.value, s[0].a)
        assertEquals(w.tick, s[0].tick)
        assertTrue(s[0].value > 0.0)
        w.setIntent(b, LocomotionIntent())
        w.step()
        assertEquals(0, slips(w).size)
    }

    @Test fun slipExcessIsRequestedMinusAllowed() {
        // Pure braking at top speed: requested maxDecel, allowed traction·g.
        val ap = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 12.0, turnRateAtRest = 10.0, turnRateAtMax = 2.0, traction = 0.9)
        val w = world()
        val p = runner(w, vx = 9.0, ap = ap)
        w.setIntent(p, LocomotionIntent.velocity(0.0, 0.0))
        w.step()
        val s = slips(w)
        assertEquals(1, s.size)
        assertEquals(12.0 - 0.9 * 9.81, s[0].value, 1e-9)
        assertEquals(9.0 - 0.9 * 9.81 * dt, speed(w, p), 1e-9)
    }

    @Test fun frozenSurfaceLowersTheLimit() {
        assertTrue(SurfaceConfig.FROZEN.tractionMultiplier < SurfaceConfig.WET.tractionMultiplier)
        assertTrue(SurfaceConfig.WET.tractionMultiplier < SurfaceConfig.NATURAL_GRASS.tractionMultiplier)
        assertEquals(1.0, SurfaceConfig.ARTIFICIAL_TURF.tractionMultiplier)
        val w = world()
        assertSame(SurfaceConfig.ARTIFICIAL_TURF, w.surface)
        w.setSurface(SurfaceConfig.FROZEN)
        assertSame(SurfaceConfig.FROZEN, w.surface)
        val p = runner(w, x = 10.0, ap = grippy.copy(traction = 0.9))
        w.setIntent(p, LocomotionIntent.velocity(9.0, 0.0))
        w.step()
        assertEquals(0.9 * SurfaceConfig.FROZEN.tractionMultiplier * 9.81 * dt, speed(w, p), 1e-9)
    }

    // ------------------------------------------------------------ contact

    @Test fun playersDriveThroughContact() {
        // A runner pushing a resting, heavier player with no intent moves him: the drive is a real force.
        val w = world()
        val p = runner(w, x = 30.0)
        val wall = w.createBody(BodyDef(x = 30.7, y = 24.0, mass = 140.0, shape = ShapeDef.Circle(0.36)))
        w.setIntent(p, LocomotionIntent.velocity(5.0, 0.0))
        repeat(120) { w.step() }
        assertTrue(w.body(wall).x > 31.5, "the pushed player moved: ${w.body(wall).x}")
        assertTrue(w.body(p).x < w.body(wall).x)
    }

    // ------------------------------------------------------------ stance

    @Test fun stanceSwapsTheColliderInPlace() {
        val w = world()
        val p = runner(w)
        val circle = w.body(p).shape as ShapeDef.Circle
        val mass = w.body(p).mass
        val uprightInertia = w.body(p).inertia
        assertEquals(Stance.Upright, w.body(p).stance)

        w.setStance(p, Stance.Diving)
        val diving = w.body(p).shape
        assertTrue(diving is ShapeDef.Capsule)
        assertTrue(diving.radius < circle.radius && diving.halfLength > circle.radius)
        assertEquals(Stance.Diving, w.body(p).stance)
        assertEquals(Material.PLAYER, w.body(p).material)
        assertEquals(mass, w.body(p).mass)
        assertTrue(w.body(p).inertia > uprightInertia, "lying down is harder to spin")

        w.setStance(p, Stance.Down)
        assertTrue(w.body(p).shape is ShapeDef.Capsule)
        assertEquals(Material.DOWNED_PLAYER, w.body(p).material)

        w.setStance(p, Stance.Upright)
        assertSame(circle, w.body(p).shape)
        assertEquals(Material.PLAYER, w.body(p).material)
        assertEquals(uprightInertia, w.body(p).inertia)
    }

    @Test fun stanceKeepsTheGamesOwnMaterial() {
        val w = world()
        val p = w.createBody(BodyDef(x = 30.0, y = 24.0, material = 6, shape = ShapeDef.Circle(0.33)))
        w.setStance(p, Stance.Down)
        assertEquals(Material.DOWNED_PLAYER, w.body(p).material)
        w.setStance(p, Stance.Upright)
        assertEquals(6, w.body(p).material)
    }

    @Test fun capsuleLiesAlongTheBodyAngle() {
        for (along in listOf(true, false)) {
            val w = world()
            // Lying along +y, the capsule reaches 0.8 m up; lying along +x it does not.
            val p = w.createBody(BodyDef(x = 30.0, y = 24.0, angle = if (along) DMath.PI / 2 else 0.0, shape = ShapeDef.Circle(0.33)))
            val other = w.createBody(BodyDef(x = 30.0, y = 25.0, shape = ShapeDef.Circle(0.3)))
            w.setStance(p, Stance.Down)
            w.step()
            var touching = false
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                if (c.bodyA == p && c.wall == ContactView.NO_WALL && c.bodyB == other && c.touching) touching = true
            }
            assertEquals(along, touching, "capsule along ${if (along) "y" else "x"}")
        }
    }

    @Test fun downedPlayerIgnoresTheIntentAndSlidesToAStop() {
        val w = world()
        val p = runner(w, vx = 6.0)
        w.setIntent(p, LocomotionIntent.velocity(9.0, 0.0))
        w.setStance(p, Stance.Down)
        var last = speed(w, p)
        repeat(120) {
            w.step()
            assertTrue(speed(w, p) <= last + 1e-12, "a downed player never speeds up")
            last = speed(w, p)
        }
        assertEquals(0.0, speed(w, p), 1e-9)
    }

    @Test fun diverSlidesStraightAndSlowsDown() {
        val w = world()
        val p = runner(w, vx = 8.0)
        w.setIntent(p, LocomotionIntent.velocity(0.0, 9.0)) // the intent wants to turn; a diver can't
        w.setStance(p, Stance.Diving)
        repeat(20) { w.step() }
        assertEquals(0.0, w.body(p).vy, 1e-12)
        // Sliding friction is the ground's, not the athlete's cleat traction (grippy has traction 10).
        assertEquals(8.0 - w.config.slideFriction * w.config.gravity * 20 * dt, w.body(p).vx, 1e-9)
    }

    @Test fun intentStanceIsAppliedAndNullKeepsTheCurrentStance() {
        val w = world()
        val p = runner(w)
        w.setIntent(p, LocomotionIntent.velocity(1.0, 0.0).apply { stance = Stance.Down })
        assertEquals(Stance.Down, w.body(p).stance)
        w.setIntent(p, LocomotionIntent.velocity(1.0, 0.0))
        assertEquals(Stance.Down, w.body(p).stance)
    }

    @Test fun stanceNeedsADynamicCircleBody() {
        val w = world()
        val capsule = w.createBody(BodyDef(shape = ShapeDef.Capsule(0.5, 0.3)))
        assertFailsWith<IllegalArgumentException> { w.setStance(capsule, Stance.Down) }
        val static = w.createBody(BodyDef(type = BodyType.Static, x = 5.0, shape = ShapeDef.Circle(0.3)))
        assertFailsWith<IllegalArgumentException> { w.setStance(static, Stance.Down) }
    }

    // ------------------------------------------------------------ waking

    @Test fun intentAndStanceWakeSleepers() {
        val w = world()
        val a = runner(w)
        val b = w.createBody(BodyDef(x = 40.0, y = 24.0, shape = ShapeDef.Circle(0.33)))
        repeat(60) { w.step() }
        assertFalse(w.body(a).awake); assertFalse(w.body(b).awake)
        w.setIntent(a, LocomotionIntent.velocity(3.0, 0.0))
        w.setStance(b, Stance.Down)
        assertTrue(w.body(a).awake); assertTrue(w.body(b).awake)
        repeat(30) { w.step() }
        assertTrue(w.body(a).vx > 1.0, "the woken runner moves")
    }

    @Test fun anActiveIntentKeepsTheBodyAwake() {
        val w = world()
        val p = runner(w)
        w.setIntent(p, LocomotionIntent.velocity(0.0, 0.0)) // "stand still" is still an intent
        repeat(120) { w.step() }
        // Standing still with zero velocity may sleep; a nonzero intent must never fall asleep mid-run.
        w.setIntent(p, LocomotionIntent.velocity(0.01, 0.0))
        repeat(120) { w.step() }
        assertTrue(w.body(p).awake)
    }

    // ------------------------------------------------------------ validation

    @Test fun rejectsBadAthleteParams() {
        assertFailsWith<IllegalArgumentException> { AthleteParams(maxSpeed = 0.0) }
        assertFailsWith<IllegalArgumentException> { AthleteParams(maxAccel = Double.NaN) }
        assertFailsWith<IllegalArgumentException> { AthleteParams(maxDecel = -1.0) }
        assertFailsWith<IllegalArgumentException> { AthleteParams(traction = 0.0) }
        assertFailsWith<IllegalArgumentException> { AthleteParams(turnRateAtRest = 1.0, turnRateAtMax = 2.0) }
        assertFailsWith<IllegalArgumentException> { AthleteParams(balance = 1.5) }
        assertFailsWith<IllegalArgumentException> { SurfaceConfig("x", tractionMultiplier = 0.0, ballBounceMultiplier = 1.0) }
    }

    @Test fun rejectsBadIntents() {
        val w = world()
        val p = w.createBody(BodyDef())
        assertFailsWith<IllegalStateException> { w.setIntent(p, LocomotionIntent.velocity(1.0, 0.0)) }
        w.setAthlete(p, AthleteParams())
        assertFailsWith<IllegalArgumentException> { w.setIntent(p, LocomotionIntent.velocity(1.0, 0.0, effort = 1.5)) }
        assertFailsWith<IllegalArgumentException> { w.setIntent(p, LocomotionIntent.velocity(Double.NaN, 0.0)) }
        assertFailsWith<IllegalArgumentException> { w.setIntent(p, LocomotionIntent.point(1.0, Double.POSITIVE_INFINITY)) }
        val static = w.createBody(BodyDef(type = BodyType.Static, x = 5.0))
        assertFailsWith<IllegalArgumentException> { w.setAthlete(static, AthleteParams()) }
    }

    // ------------------------------------------------------------ API: views, recorder

    @Test fun bodyViewShowsTheAthlete() {
        val w = world()
        val p = w.createBody(BodyDef())
        assertEquals(null, w.body(p).athlete)
        w.setAthlete(p, grippy)
        assertSame(grippy, w.body(p).athlete)
    }

    @Test fun intentIsCopiedNotKept() {
        val w = world()
        val p = runner(w, x = 10.0)
        val intent = LocomotionIntent.velocity(5.0, 0.0)
        w.setIntent(p, intent)
        intent.setVelocity(0.0, 5.0) // reusing the object must not change the body's intent
        repeat(300) { w.step() }
        assertEquals(5.0, w.body(p).vx, 0.05); assertEquals(0.0, w.body(p).vy, 1e-12)
    }

    @Test fun recorderSeesEveryLocomotionCall() {
        val log = mutableListOf<String>()
        val w = world()
        w.recorder = object : CommandRecorder {
            override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) {}
            override fun onDestroyBody(tick: Long, id: BodyId) {}
            override fun onSetAthlete(tick: Long, id: BodyId, params: AthleteParams) { log += "a:${id.value}:${params.maxSpeed}@$tick" }
            override fun onSetIntent(tick: Long, id: BodyId, intent: LocomotionIntent) {
                val copy = LocomotionIntent().copyFrom(intent)
                log += "i:${id.value}:${copy.mode}:${copy.targetX},${copy.targetY}:${copy.effort}:${copy.stance}@$tick"
            }
            override fun onSetStance(tick: Long, id: BodyId, stance: Stance) { log += "s:${id.value}:$stance@$tick" }
            override fun onSetSurface(tick: Long, surface: SurfaceConfig) { log += "f:${surface.name}@$tick" }
            override fun onCreateGrapple(tick: Long, id: JointId, tackler: BodyId, carrier: BodyId, def: GrappleDef) {}
            override fun onCreateBlock(tick: Long, id: JointId, blocker: BodyId, defender: BodyId, def: BlockDef) {}
            override fun onCreateDistance(tick: Long, id: JointId, a: BodyId, b: BodyId, def: DistanceDef) {}
            override fun onCreateWeld(tick: Long, id: JointId, a: BodyId, b: BodyId, def: WeldDef) {}
            override fun onDestroyJoint(tick: Long, id: JointId) {}
            override fun onSetBallCarrier(tick: Long, id: BodyId?) {}
            override fun onSetAttackDirection(tick: Long, direction: Int) {}
        }
        val p = w.createBody(BodyDef())
        w.setAthlete(p, AthleteParams(maxSpeed = 8.0))
        w.step()
        w.setIntent(p, LocomotionIntent.point(3.0, 4.0, effort = 0.5))
        w.setStance(p, Stance.Diving)
        w.step()
        w.setSurface(SurfaceConfig.WET)
        assertEquals(
            listOf("a:0:8.0@0", "i:0:Point:3.0,4.0:0.5:null@1", "s:0:Diving@1", "f:Wet@2"),
            log,
        )
    }

    // ------------------------------------------------------------ determinism and allocation

    private fun randomPlay(seed: Long, ticks: Int, hashes: LongArray?) {
        val w = world()
        val rng = SplitMix64(seed)
        val ids = ArrayList<BodyId>()
        for (k in 0 until 12) {
            val id = w.createBody(BodyDef(x = 40.0 + rng.nextDouble(0.0, 20.0), y = 14.0 + rng.nextDouble(0.0, 20.0), mass = rng.nextDouble(85.0, 150.0), shape = ShapeDef.Circle(rng.nextDouble(0.30, 0.38))))
            w.setAthlete(id, AthleteParams(maxSpeed = rng.nextDouble(6.0, 10.0), maxAccel = rng.nextDouble(5.0, 8.0)))
            ids += id
        }
        val intent = LocomotionIntent()
        for (t in 0 until ticks) {
            if (t % 20 == 0) {
                for (id in ids) {
                    if (rng.nextDouble(0.0, 1.0) < 0.5) intent.setPoint(rng.nextDouble(30.0, 70.0), rng.nextDouble(5.0, 44.0), rng.nextDouble(0.3, 1.0))
                    else intent.setVelocity(rng.nextDouble(-9.0, 9.0), rng.nextDouble(-9.0, 9.0), 1.0)
                    w.setIntent(id, intent)
                    if (rng.nextDouble(0.0, 1.0) < 0.05) w.setStance(id, Stance.entries[(rng.nextLong() and 1L).toInt() + 1])
                    else if (rng.nextDouble(0.0, 1.0) < 0.3) w.setStance(id, Stance.Upright)
                }
            }
            w.step()
            if (hashes != null) hashes[t] = w.stateHash()
        }
    }

    @Test fun locomotionIsDeterministic() {
        val a = LongArray(900); val b = LongArray(900)
        randomPlay(42L, 900, a)
        randomPlay(42L, 900, b)
        assertTrue(a.contentEquals(b))
        val c = LongArray(900)
        randomPlay(43L, 900, c)
        assertNotEquals(a.last(), c.last())
    }

    @Test fun stepDoesNotAllocateWithLocomotionAndContacts() {
        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        val w = world()
        val ids = ArrayList<BodyId>()
        // Two packs running into each other: locomotion, contacts, slips and turning every tick.
        for (k in 0 until 6) {
            val a = runner(w, x = 40.0, y = 20.0 + k * 0.8, ap = grippy.copy(traction = 0.9))
            val b = runner(w, x = 46.0, y = 20.3 + k * 0.8, ap = grippy.copy(traction = 0.9))
            w.setIntent(a, LocomotionIntent.point(48.0, 22.0))
            w.setIntent(b, LocomotionIntent.velocity(-9.0, 2.0))
            ids += a; ids += b
        }
        // Warm up (class loading, JIT): back and forth so the packs keep meeting and stay on the field.
        for (round in 0 until 6) {
            for (id in ids) w.setIntent(id, LocomotionIntent.point(if (round % 2 == 0) 50.0 else 38.0, 22.0))
            repeat(250) { w.step() }
        }
        for (id in ids) w.setIntent(id, LocomotionIntent.velocity(9.0, -3.0))
        val tid = Thread.currentThread().id
        val before = mx.getThreadAllocatedBytes(tid)
        for (t in 0 until 300) w.step()
        val allocated = mx.getThreadAllocatedBytes(tid) - before
        assertEquals(0L, allocated, "bytes allocated by 300 steps")
    }
}
