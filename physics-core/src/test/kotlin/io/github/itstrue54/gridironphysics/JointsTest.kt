package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class JointsTest {
    private val f = FieldConfig.PRO
    private val dt = WorldConfig.DT

    private fun world(config: WorldConfig = WorldConfig()) = World(config, f, 1L)

    /** A body that touches nothing but the runoff walls, so only the joint acts on it. */
    private fun ghost(
        w: World, x: Double, y: Double = 24.0, vx: Double = 0.0, vy: Double = 0.0, angle: Double = 0.0,
        mass: Double = 100.0, type: BodyType = BodyType.Dynamic,
    ): BodyId = w.createBody(
        BodyDef(
            type = type, x = x, y = y, vx = vx, vy = vy, angle = angle, mass = mass,
            shape = ShapeDef.Circle(0.34), maskBits = Category.STATIC,
        ),
    )

    private fun dist(w: World, a: BodyId, b: BodyId): Double {
        val ax = w.body(a).x; val ay = w.body(a).y
        val bx = w.body(b).x; val by = w.body(b).y
        return DMath.hypot2(bx - ax, by - ay)
    }

    private fun momentumX(w: World, ids: List<BodyId>): Double = ids.sumOf { w.body(it).mass * w.body(it).vx }

    private val puller = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)

    // ------------------------------------------------------------ grapple

    @Test fun grappleHoldsTheCarrierAtItsLengthAndConservesMomentum() {
        // Real players: the rope only pulls, so it is the contact that stops the recoil (a pull-only rope cannot damp
        // the approach), just as a wrapped-up tackler bumps into the carrier.
        val w = world()
        fun player(x: Double, vx: Double) = w.createBody(BodyDef(x = x, y = 24.0, vx = vx, shape = ShapeDef.Circle(0.34)))
        val tackler = player(50.0, 0.0)
        val carrier = player(50.7, 3.0)
        val j = w.createGrapple(tackler, carrier, GrappleDef(breakForce = 1e6))
        assertEquals(0.7, w.joint(j).restLength, 1e-12)
        val p0 = momentumX(w, listOf(tackler, carrier))
        repeat(180) { w.step() }
        assertTrue(w.hasJoint(j))
        assertTrue(dist(w, tackler, carrier) in 0.67..0.75, "held between touching and its length: ${dist(w, tackler, carrier)}")
        assertEquals(p0, momentumX(w, listOf(tackler, carrier)), 1e-9, "a joint is internal: momentum is conserved")
        assertEquals(1.5, w.body(tackler).vx, 0.05, "they end up moving together")
        assertEquals(1.5, w.body(carrier).vx, 0.05)
    }

    @Test fun grappleIsTensionOnly() {
        val w = world()
        val tackler = ghost(w, 50.0)
        val carrier = ghost(w, 51.5, vx = -1.0) // moving closer than the rest length: slack
        val j = w.createGrapple(tackler, carrier, GrappleDef(breakForce = 1e6, length = 1.5))
        repeat(30) { w.step() }
        assertEquals(-1.0, w.body(carrier).vx, 1e-12)
        assertEquals(0.0, w.body(tackler).vx, 1e-12)
        assertEquals(0.0, w.joint(j).force)
    }

    @Test fun grappleHoldsWhileTheTensionStaysBelowItsBreakForce() {
        // A 100 kg carrier pulls about 700 N at first; dragging an equal tackler needs about half of that.
        val w = world()
        val tackler = ghost(w, 50.0)
        val carrier = ghost(w, 50.7)
        w.setAthlete(carrier, puller)
        w.setIntent(carrier, LocomotionIntent.velocity(9.0, 0.0))
        val j = w.createGrapple(tackler, carrier, GrappleDef(breakForce = 500.0))
        var peak = 0.0
        repeat(180) { w.step(); peak = maxOf(peak, w.joint(j).force) }
        assertTrue(w.hasJoint(j), "held")
        assertTrue(peak in 250.0..500.0, "peak tension $peak")
        assertTrue(w.drain().none { it.type == EventType.GrappleBroken })
    }

    @Test fun grappleBreaksAndReportsItsBodiesAndForce() {
        val w = world()
        val tackler = ghost(w, 50.0)
        val carrier = ghost(w, 50.7)
        w.setAthlete(carrier, puller)
        w.setIntent(carrier, LocomotionIntent.velocity(9.0, 0.0))
        val j = w.createGrapple(tackler, carrier, GrappleDef(breakForce = 200.0))
        val evs = w.run(60).filter { it.type == EventType.GrappleBroken }
        assertEquals(1, evs.size, "exactly one break")
        val e = evs.single()
        assertEquals(tackler.value, e.a); assertEquals(carrier.value, e.b)
        assertEquals(j.value, e.joint)
        assertEquals(200.0, e.value, 1e-9, "the joint never carries more than its break force")
        assertFalse(w.hasJoint(j))
        assertEquals(0, w.jointCount)
        assertFailsWith<IllegalArgumentException> { w.destroyJoint(j) }
        // Free: the carrier now accelerates away from the tackler.
        val gap = dist(w, tackler, carrier)
        repeat(30) { w.step() }
        assertTrue(dist(w, tackler, carrier) > gap + 0.5)
    }

    @Test fun dragSlowsTheCarrierAndStacksAcrossGrapples() {
        for (n in 1..3) {
            val w = world()
            val carrier = ghost(w, 50.0, vx = 6.0)
            val tacklers = (0 until n).map { k -> ghost(w, 49.3, y = 23.0 + k, vx = 6.0) }
            for (t in tacklers) w.createGrapple(t, carrier, GrappleDef(breakForce = 1e6, dragForce = 400.0, length = 2.0))
            val all = tacklers + carrier
            val p0 = momentumX(w, all)
            w.step()
            assertEquals(n * 400.0 * dt, p0 - momentumX(w, all), 1e-9, "$n grapples take $n × dragForce × dt of momentum per tick")
        }
    }

    @Test fun dragStopsTheCarrierButNeverPushesHimBackward() {
        val w = world()
        val tackler = ghost(w, 48.0, type = BodyType.Static)
        val carrier = ghost(w, 50.0, vx = 0.5)
        w.createGrapple(tackler, carrier, GrappleDef(breakForce = 1e6, dragForce = 5000.0, length = 3.0))
        w.step()
        assertEquals(0.0, w.body(carrier).vx, 1e-12)
        repeat(10) { w.step() }
        assertEquals(0.0, w.body(carrier).vx, 1e-12)
        assertEquals(50.0, w.body(carrier).x, 1e-12)
    }

    @Test fun grappleWakesASleepingTacklerWhenTheCarrierMoves() {
        val w = world()
        val tackler = ghost(w, 50.0)
        val carrier = ghost(w, 50.7)
        w.setAthlete(carrier, puller)
        w.createGrapple(tackler, carrier, GrappleDef(breakForce = 1e6))
        repeat(60) { w.step() }
        assertFalse(w.body(tackler).awake); assertFalse(w.body(carrier).awake)
        w.setIntent(carrier, LocomotionIntent.velocity(5.0, 0.0))
        repeat(30) { w.step() }
        assertTrue(w.body(tackler).awake, "the joint wakes its other body")
        assertTrue(w.body(tackler).x > 50.05, "and drags it along")
    }

    // ------------------------------------------------------------ block

    /** Blocker at x = 50 facing +x, defender at x = 50.7 facing −x: squared up. */
    private fun block(
        w: World, blockerPush: Double, defenderPush: Double, shed: Double = 1e6,
        blockerAngle: Double = 0.0, driveSpeed: Double = 2.5,
    ): Triple<BodyId, BodyId, JointId> {
        val blocker = ghost(w, 50.0, angle = blockerAngle, mass = 140.0)
        val defender = ghost(w, 50.7, angle = DMath.PI, mass = 150.0)
        val j = w.createBlock(
            blocker, defender,
            BlockDef(blockerPushForce = blockerPush, defenderPushForce = defenderPush, shedThreshold = shed, driveSpeed = driveSpeed),
        )
        return Triple(blocker, defender, j)
    }

    @Test fun theStrongerSideDrivesThePair() {
        for ((bp, dp, sign) in listOf(Triple(1500.0, 1000.0, 1.0), Triple(1000.0, 1500.0, -1.0))) {
            val w = world()
            val (blocker, defender, j) = block(w, bp, dp)
            repeat(60) { w.step() }
            assertTrue(w.hasJoint(j))
            assertTrue(sign * w.body(blocker).vx > 0.2 && sign * w.body(defender).vx > 0.2, "pushes $bp vs $dp drive toward ${if (sign > 0) "+x" else "-x"}")
            assertEquals(0.7, dist(w, blocker, defender), 0.03, "engaged")
        }
    }

    @Test fun equalPushesStandStill() {
        val w = world()
        val (blocker, defender, _) = block(w, 1200.0, 1200.0)
        fun com() = (140.0 * w.body(blocker).x + 150.0 * w.body(defender).x) / 290.0
        val c0 = com()
        repeat(120) { w.step() }
        assertEquals(c0, com(), 1e-3, "neither side gains ground")
        assertEquals(0.0, w.body(blocker).vx, 1e-3)
        assertEquals(0.0, w.body(defender).vx, 1e-3)
    }

    @Test fun pushTapersWithSpeedSoTheDriveLevelsOff() {
        val w = world()
        val (blocker, _, j) = block(w, 2000.0, 1000.0, driveSpeed = 2.0)
        repeat(600) { w.step() }
        // Steady state: 2000·(1 − v/2) = 1000 → v = 1 m/s (the defender is pushed backward, so his push stays full).
        assertEquals(1.0, w.body(blocker).vx, 0.02)
        assertEquals(1000.0, w.joint(j).blockerPush, 20.0)
        assertEquals(1000.0, w.joint(j).defenderPush, 1e-9)
    }

    @Test fun leverageScalesThePushByTheCosineOfTheHeading() {
        for ((angle, expected) in listOf(0.0 to 1500.0, DMath.PI / 3 to 750.0, -DMath.PI / 3 to 750.0, DMath.PI / 2 to 0.0, DMath.PI to 0.0)) {
            val w = world()
            val (_, _, j) = block(w, 1500.0, 0.0, blockerAngle = angle)
            w.step()
            assertEquals(expected, w.joint(j).blockerPush, 1e-6, "heading $angle")
        }
    }

    @Test fun lateralForceAboveTheShedThresholdShedsTheBlock() {
        for ((shed, sheds) in listOf(300.0 to true, 2000.0 to false)) {
            val w = world()
            val (blocker, defender, j) = block(w, 1000.0, 1000.0, shed = shed)
            // The defender rips sideways: ~7 m/s² × 150 kg ≈ 1050 N across the joint line, about half of which the
            // joint needs to drag the blocker along.
            w.setAthlete(defender, puller)
            w.setIntent(defender, LocomotionIntent.velocity(0.0, 6.0))
            val evs = w.run(60)
            val shedEv = evs.filter { it.type == EventType.BlockShed }
            if (sheds) {
                assertEquals(1, shedEv.size)
                val e = shedEv.single()
                assertEquals(blocker.value, e.a); assertEquals(defender.value, e.b); assertEquals(j.value, e.joint)
                assertEquals(shed, e.value, 1e-9)
                assertFalse(w.hasJoint(j))
            } else {
                assertTrue(shedEv.isEmpty())
                assertTrue(w.hasJoint(j))
                assertTrue(w.joint(j).lateralForce < shed)
            }
        }
    }

    @Test fun doubleTeamOutpushesAStrongerDefender() {
        // 150 kg lineman with 2400 N of push against two blockers with 1500 N each, angled in at ±30°.
        val w = world()
        val lineman = ghost(w, 50.7, angle = DMath.PI, mass = 150.0)
        val c = DMath.cos(DMath.PI / 6); val s = DMath.sin(DMath.PI / 6)
        val b1 = ghost(w, 50.7 - 0.7 * c, y = 24.0 - 0.7 * s, angle = DMath.PI / 6, mass = 140.0)
        val b2 = ghost(w, 50.7 - 0.7 * c, y = 24.0 + 0.7 * s, angle = -DMath.PI / 6, mass = 135.0)
        for (b in listOf(b1, b2)) w.createBlock(b, lineman, BlockDef(blockerPushForce = 1500.0, defenderPushForce = 1200.0, shedThreshold = 800.0))
        repeat(120) { w.step() }
        assertEquals(2, w.jointCount, "neither block is shed")
        assertTrue(w.body(lineman).x > 51.0, "the lineman is driven back: x = ${w.body(lineman).x}")
        assertEquals(24.0, w.body(lineman).y, 0.05, "a near-symmetric double team keeps him on his line")
    }

    // ------------------------------------------------------------ distance and weld

    @Test fun rigidDistanceKeepsItsLengthInBothArgumentOrders() {
        val results = (0..1).map { order ->
            val w = world()
            val a = ghost(w, 50.0, vx = -2.0)
            val b = ghost(w, 51.0, vx = 2.0, vy = 1.0)
            val j = if (order == 0) w.createDistance(a, b, DistanceDef()) else w.createDistance(b, a, DistanceDef())
            var worst = 0.0
            repeat(120) { w.step(); worst = maxOf(worst, DMath.abs(dist(w, a, b) - 1.0)) }
            assertTrue(worst < 0.01, "length error $worst")
            assertTrue(w.joint(j).force >= 0.0)
            listOf(w.body(a).x, w.body(a).y, w.body(b).x, w.body(b).y)
        }
        for (k in 0 until 4) assertEquals(results[0][k], results[1][k], 1e-6)
    }

    @Test fun softDistanceIsASpringBackToItsLength() {
        val w = world()
        val a = ghost(w, 50.0)
        val b = ghost(w, 52.0)
        w.createDistance(a, b, DistanceDef(length = 1.0, hertz = 2.0, dampingRatio = 1.0))
        w.step()
        assertTrue(dist(w, a, b) > 1.9, "a soft spring does not snap")
        repeat(240) { w.step() }
        assertEquals(1.0, dist(w, a, b), 0.01)
    }

    @Test fun distanceToAStaticBodyIsAPendulum() {
        val w = world()
        val pivot = ghost(w, 50.0, type = BodyType.Static)
        val bob = ghost(w, 51.0, vy = 3.0)
        w.createDistance(pivot, bob, DistanceDef())
        repeat(120) { w.step() }
        assertEquals(1.0, dist(w, pivot, bob), 0.01)
        assertEquals(50.0, w.body(pivot).x)
    }

    @Test fun weldKeepsTheRelativePoseAsTheBodiesMoveAndTurn() {
        val w = world()
        val a = ghost(w, 50.0, angle = 0.3)
        val b = ghost(w, 50.5, y = 24.4, angle = 1.0)
        w.createWeld(a, b, WeldDef())
        // Local offset of b in a's frame, and the relative angle, at creation.
        fun local(): Triple<Double, Double, Double> {
            val ax = w.body(a).x; val ay = w.body(a).y; val aa = w.body(a).angle
            val bx = w.body(b).x; val by = w.body(b).y; val ba = w.body(b).angle
            val c = DMath.cos(aa); val s = DMath.sin(aa)
            val dx = bx - ax; val dy = by - ay
            return Triple(c * dx + s * dy, -s * dx + c * dy, ba - aa)
        }
        val l0 = local()
        // Shove b sideways with a pull from a third body on a spring.
        val anchor = ghost(w, 50.5, y = 30.0, type = BodyType.Static)
        w.createDistance(anchor, b, DistanceDef(length = 1.0, hertz = 1.0))
        repeat(120) { w.step() }
        val l = local()
        assertTrue(DMath.abs(w.body(a).angle - 0.3) > 0.05, "the pull turned the pair")
        assertEquals(l0.first, l.first, 0.01); assertEquals(l0.second, l.second, 0.01); assertEquals(l0.third, l.third, 0.01)
    }

    // ------------------------------------------------------------ lifecycle

    @Test fun destroyJointStopsItAndStaleIdsAreRejected() {
        val w = world()
        val a = ghost(w, 50.0)
        val b = ghost(w, 51.0, vx = 2.0)
        val j = w.createDistance(a, b, DistanceDef())
        assertEquals(1, w.jointCount)
        w.step()
        w.destroyJoint(j)
        assertFalse(w.hasJoint(j))
        assertFailsWith<IllegalArgumentException> { w.joint(j) }
        assertFailsWith<IllegalArgumentException> { w.destroyJoint(j) }
        val vb = w.body(b).vx; val va = w.body(a).vx
        repeat(10) { w.step() }
        assertEquals(va, w.body(a).vx); assertEquals(vb, w.body(b).vx)
        val j2 = w.createDistance(a, b, DistanceDef())
        assertTrue(j2 != j, "a reused slot gets a new generation")
        assertFalse(w.hasJoint(j))
    }

    @Test fun destroyingABodyDestroysItsJoints() {
        val w = world()
        val a = ghost(w, 50.0)
        val b = ghost(w, 51.0)
        val c = ghost(w, 52.0)
        val ab = w.createDistance(a, b, DistanceDef())
        val bc = w.createGrapple(b, c, GrappleDef(breakForce = 100.0))
        val ac = w.createWeld(a, c, WeldDef())
        w.destroyBody(b)
        assertFalse(w.hasJoint(ab)); assertFalse(w.hasJoint(bc)); assertTrue(w.hasJoint(ac))
        assertEquals(1, w.jointCount)
        repeat(5) { w.step() }
        assertTrue(w.drain().none { it.type == EventType.GrappleBroken }, "destroying is not breaking")
    }

    @Test fun jointIdsAreListedInIdOrder() {
        val w = world()
        val a = ghost(w, 50.0); val b = ghost(w, 51.0); val c = ghost(w, 52.0)
        val j0 = w.createDistance(a, b, DistanceDef())
        val j1 = w.createDistance(b, c, DistanceDef())
        val j2 = w.createDistance(a, c, DistanceDef())
        w.destroyJoint(j1)
        val j3 = w.createDistance(b, c, DistanceDef()) // reuses slot 1
        val out = IntArray(8)
        assertEquals(3, w.jointIds(out))
        assertEquals(listOf(j0.value, j3.value, j2.value), out.take(3))
        val v = w.joint(j3)
        assertEquals(JointType.Distance, v.type); assertEquals(b, v.bodyA); assertEquals(c, v.bodyB)
    }

    @Test fun rejectsBadJoints() {
        val w = world()
        val a = ghost(w, 50.0)
        val b = ghost(w, 51.0)
        val s1 = ghost(w, 52.0, type = BodyType.Static)
        val s2 = ghost(w, 53.0, type = BodyType.Static)
        assertFailsWith<IllegalArgumentException> { w.createDistance(a, a, DistanceDef()) }
        assertFailsWith<IllegalArgumentException> { w.createDistance(s1, s2, DistanceDef()) }
        assertFailsWith<IllegalArgumentException> { w.createGrapple(a, s1, GrappleDef(breakForce = 10.0)) } // carrier must be dynamic
        assertFailsWith<IllegalArgumentException> { w.createDistance(a, BodyId(12345 shl 20), DistanceDef()) }
        assertFailsWith<IllegalArgumentException> { GrappleDef(breakForce = 0.0) }
        assertFailsWith<IllegalArgumentException> { GrappleDef(breakForce = Double.NaN) }
        assertFailsWith<IllegalArgumentException> { GrappleDef(breakForce = 10.0, dragForce = -1.0) }
        assertFailsWith<IllegalArgumentException> { GrappleDef(breakForce = 10.0, hertz = 0.0) }
        assertFailsWith<IllegalArgumentException> { BlockDef(blockerPushForce = -1.0, defenderPushForce = 0.0, shedThreshold = 1.0) }
        assertFailsWith<IllegalArgumentException> { BlockDef(blockerPushForce = 1.0, defenderPushForce = 1.0, shedThreshold = 0.0) }
        assertFailsWith<IllegalArgumentException> { BlockDef(blockerPushForce = 1.0, defenderPushForce = 1.0, shedThreshold = 1.0, driveSpeed = 0.0) }
        assertFailsWith<IllegalArgumentException> { DistanceDef(length = 0.0) }
        assertFailsWith<IllegalArgumentException> { DistanceDef(hertz = -1.0) }
        assertFailsWith<IllegalArgumentException> { WeldDef(dampingRatio = -1.0) }
        val twin = ghost(w, 50.0) // same center as a
        assertFailsWith<IllegalArgumentException> { w.createDistance(a, twin, DistanceDef()) }
        w.createDistance(a, twin, DistanceDef(length = 0.7)) // fine with an explicit length
        w.destroyJoint(JointId(0))
        GrappleDef(breakForce = Double.POSITIVE_INFINITY) // never breaks: allowed
        assertEquals(0, w.jointCount)
        assertTrue(w.hasJoint(w.createDistance(a, b, DistanceDef())))
    }

    @Test fun fullJointPoolThrows() {
        val w = world(WorldConfig(maxJoints = 2))
        val a = ghost(w, 50.0); val b = ghost(w, 51.0)
        w.createDistance(a, b, DistanceDef()); w.createDistance(a, b, DistanceDef())
        assertFailsWith<IllegalStateException> { w.createDistance(a, b, DistanceDef()) }
    }

    @Test fun recorderSeesEveryJointCommand() {
        val w = world()
        val log = mutableListOf<String>()
        w.recorder = object : CommandRecorder {
            override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) {}
            override fun onDestroyBody(tick: Long, id: BodyId) {}
            override fun onSetAthlete(tick: Long, id: BodyId, params: AthleteParams) {}
            override fun onSetIntent(tick: Long, id: BodyId, intent: LocomotionIntent) {}
            override fun onSetStance(tick: Long, id: BodyId, stance: Stance) {}
            override fun onSetSurface(tick: Long, surface: SurfaceConfig) {}
            override fun onCreateGrapple(tick: Long, id: JointId, tackler: BodyId, carrier: BodyId, def: GrappleDef) { log += "g:${tackler.value}>${carrier.value}:${def.breakForce}@$tick" }
            override fun onCreateBlock(tick: Long, id: JointId, blocker: BodyId, defender: BodyId, def: BlockDef) { log += "b:${blocker.value}>${defender.value}:${def.shedThreshold}@$tick" }
            override fun onCreateDistance(tick: Long, id: JointId, a: BodyId, b: BodyId, def: DistanceDef) { log += "d:${a.value}>${b.value}@$tick" }
            override fun onCreateWeld(tick: Long, id: JointId, a: BodyId, b: BodyId, def: WeldDef) { log += "w:${a.value}>${b.value}@$tick" }
            override fun onDestroyJoint(tick: Long, id: JointId) { log += "x:${id.value}@$tick" }
            override fun onSetBallCarrier(tick: Long, id: BodyId?) { log += "c:${id?.value}@$tick" }
            override fun onSetAttackDirection(tick: Long, direction: Int) { log += "dir:$direction@$tick" }
        }
        val a = ghost(w, 50.0); val b = ghost(w, 51.0)
        val g = w.createGrapple(a, b, GrappleDef(breakForce = 9.0))
        w.step()
        w.createBlock(a, b, BlockDef(1.0, 2.0, 3.0))
        w.createDistance(a, b, DistanceDef())
        w.createWeld(b, a, WeldDef())
        w.destroyJoint(g)
        w.setBallCarrier(b)
        w.setAttackDirection(-1)
        w.setBallCarrier(null)
        assertEquals(
            listOf("g:${a.value}>${b.value}:9.0@0", "b:${a.value}>${b.value}:3.0@1", "d:${a.value}>${b.value}@1", "w:${b.value}>${a.value}@1",
                "x:${g.value}@1", "c:${b.value}@1", "dir:-1@1", "c:null@1"),
            log,
        )
    }

    // ------------------------------------------------------------ determinism and allocation

    /** A pile: two grapples on one carrier, a double team, a weld and a distance joint, with players driving. */
    private fun jointWorld(): World {
        val w = world()
        val carrier = ghost(w, 40.0, vx = 6.0)
        w.setAthlete(carrier, puller)
        w.setIntent(carrier, LocomotionIntent.velocity(8.0, 1.0))
        val t1 = w.createBody(BodyDef(x = 39.3, y = 24.0, vx = 5.0, mass = 105.0, shape = ShapeDef.Circle(0.34), categoryBits = Category.AWAY))
        val t2 = w.createBody(BodyDef(x = 40.0, y = 24.7, vx = 5.0, mass = 110.0, shape = ShapeDef.Circle(0.34), categoryBits = Category.AWAY))
        w.createGrapple(t1, carrier, GrappleDef(breakForce = 2000.0, dragForce = 300.0))
        w.createGrapple(t2, carrier, GrappleDef(breakForce = 2000.0, dragForce = 300.0))
        val lineman = w.createBody(BodyDef(x = 60.7, y = 20.0, angle = DMath.PI, mass = 150.0, shape = ShapeDef.Circle(0.36), categoryBits = Category.AWAY))
        val b1 = w.createBody(BodyDef(x = 60.0, y = 19.6, angle = 0.4, mass = 140.0, shape = ShapeDef.Circle(0.36)))
        val b2 = w.createBody(BodyDef(x = 60.0, y = 20.4, angle = -0.4, mass = 135.0, shape = ShapeDef.Circle(0.36)))
        w.createBlock(b1, lineman, BlockDef(1500.0, 1200.0, 900.0))
        w.createBlock(b2, lineman, BlockDef(1500.0, 1200.0, 900.0))
        val sled = w.createBody(BodyDef(x = 70.0, y = 30.0, mass = 200.0, shape = ShapeDef.box(0.5, 1.0), categoryBits = Category.STATIC))
        val pusher = w.createBody(BodyDef(x = 69.2, y = 30.0, vx = 3.0, mass = 100.0, shape = ShapeDef.Circle(0.34)))
        w.createWeld(sled, pusher, WeldDef())
        val post = w.createBody(BodyDef(type = BodyType.Static, x = 80.0, y = 10.0, shape = ShapeDef.Circle(0.2), categoryBits = Category.STATIC))
        val swing = w.createBody(BodyDef(x = 81.0, y = 10.0, vy = 4.0, mass = 90.0, shape = ShapeDef.Circle(0.33)))
        w.createDistance(post, swing, DistanceDef())
        w.setBallCarrier(carrier)
        return w
    }

    @Test fun jointsAreDeterministic() {
        fun run(): Pair<List<Long>, List<Ev>> {
            val w = jointWorld()
            val hashes = ArrayList<Long>()
            val evs = ArrayList<Ev>()
            repeat(300) { w.step(); hashes += w.stateHash(); evs += w.drain() }
            return hashes to evs
        }
        assertEquals(run(), run())
    }

    @Test fun stepWithJointsDoesNotAllocate() {
        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        val out = IntArray(16)
        var sink = 0.0
        fun tick(w: World) {
            w.step()
            val e = w.events()
            while (e.next()) {
                sink += e.tick + e.bodyA.value + e.force + e.forwardProgressX + (if (e.ballCarrier) 1 else 0)
                if (e.type == EventType.GrappleBroken || e.type == EventType.BlockShed) sink += e.joint.value + e.bodyB.value
            }
            val n = w.jointIds(out)
            for (k in 0 until n) { val v = w.joint(JointId(out[k])); sink += v.force + v.lateralForce + v.load + v.anchorAX }
            sink += w.forwardProgressX
        }
        /** A grapple that breaks and a block that sheds a few ticks into the measured window. */
        fun breakers(w: World) {
            val c = ghost(w, 30.0, y = 10.0)
            w.setAthlete(c, puller)
            w.setIntent(c, LocomotionIntent.velocity(9.0, 0.0))
            w.createGrapple(ghost(w, 29.3, y = 10.0), c, GrappleDef(breakForce = 150.0))
            val d = ghost(w, 30.7, y = 14.0, angle = DMath.PI)
            w.setAthlete(d, puller)
            w.setIntent(d, LocomotionIntent.velocity(0.0, 6.0))
            w.createBlock(ghost(w, 30.0, y = 14.0), d, BlockDef(1000.0, 1000.0, 200.0))
        }
        var w = jointWorld()
        for (round in 0 until 12) {
            w = jointWorld(); breakers(w)
            repeat(200) { tick(w) }
        }
        val tid = Thread.currentThread().id
        var allocated = 0L
        for (round in 0 until 6) {
            w = jointWorld(); breakers(w) // creation allocates, so it stays outside the measured window
            val before = mx.getThreadAllocatedBytes(tid)
            repeat(150) { tick(w) }
            allocated += mx.getThreadAllocatedBytes(tid) - before
        }
        assertEquals(0L, allocated, "bytes allocated by 900 steps with joints (sink $sink)")
    }
}
