package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.Sweep3
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNotEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class BallFlightTest {
    private val f = FieldConfig.PRO
    private val g = 9.81
    private val cy = f.widthM / 2
    private val vacuum = WorldConfig(ball = BallConfig(drag = false))

    private fun world(config: WorldConfig = WorldConfig(), seed: Long = 1L) = World(config, f, seed)

    private fun launch(w: World, x: Double, y: Double, z: Double, vx: Double, vy: Double, vz: Double, mode: FlightMode = FlightMode.Spiral) =
        w.launchBall(Vec3(x, y, z), Vec3(vx, vy, vz), mode)

    private fun player(w: World, x: Double, y: Double, angle: Double = 0.0) =
        w.createBody(BodyDef(x = x, y = y, angle = angle, shape = ShapeDef.Circle(0.34)))

    /** Steps until [type] appears (or [max] ticks pass); returns every event seen so far. */
    private fun until(w: World, type: EventType, max: Int = 600): List<Ev> {
        val all = ArrayList<Ev>()
        repeat(max) {
            w.step()
            val e = w.drain()
            all += e
            if (e.any { it.type == type }) return all
        }
        return all
    }

    // ------------------------------------------------------------ states and launch

    @Test fun theBallStartsDeadAndLaunchingMakesItAirborne() {
        val w = world()
        assertEquals(BallState.Dead, w.ball().state)
        launch(w, 30.0, 20.0, 1.8, 10.0, 2.0, 8.0, FlightMode.Wobble)
        val b = w.ball()
        assertEquals(BallState.Airborne, b.state)
        assertEquals(FlightMode.Wobble, b.flightMode)
        assertEquals(30.0, b.x); assertEquals(20.0, b.y); assertEquals(1.8, b.z)
        assertEquals(10.0, b.vx); assertEquals(2.0, b.vy); assertEquals(8.0, b.vz)
    }

    @Test fun theBallFallsUnderGravityExactlyInVacuum() {
        val w = world(vacuum)
        launch(w, 30.0, 20.0, 2.0, 12.0, -3.0, 9.0)
        repeat(30) { w.step() }
        val t = 30 * WorldConfig.DT
        val b = w.ball()
        assertEquals(30.0 + 12.0 * t, b.x, 1e-9)
        assertEquals(20.0 - 3.0 * t, b.y, 1e-9)
        assertEquals(2.0 + 9.0 * t - 0.5 * g * t * t, b.z, 1e-9)
        assertEquals(9.0 - g * t, b.vz, 1e-9)
        assertEquals(b.x - 12.0 * WorldConfig.DT, b.prevX, 1e-9)
    }

    @Test fun dragIsQuadraticInTheAirSpeed() {
        for (mode in FlightMode.entries) {
            val w = world()
            launch(w, 30.0, 20.0, 100.0, 30.0, 0.0, 0.0, mode)
            w.step()
            val c = BallConfig()
            val k = 0.5 * c.airDensity * c.cd(mode) * c.crossSection / c.mass
            // The tick's average: the ball slows within it, so allow k·v·dt relative (≈ 2e-3 at most here).
            assertEquals(-k * 30.0 * 30.0, (w.ball().vx - 30.0) / WorldConfig.DT, 1e-2 * k * 900.0, "$mode")
        }
    }

    @Test fun windCarriesTheBall() {
        fun land(windX: Double, windY: Double): Ev {
            val w = world()
            w.setWind(windX, windY)
            launch(w, 30.0, 24.0, 1.8, 18.0, 0.0, 12.0)
            return until(w, EventType.BallBounce).last { it.type == EventType.BallBounce }
        }
        val calm = land(0.0, 0.0)
        assertEquals(24.0, calm.py, 1e-9)
        assertTrue(land(0.0, 6.0).py > 24.5, "a crosswind drifts the ball sideways")
        assertTrue(land(0.0, -6.0).py < 23.5)
        assertTrue(land(8.0, 0.0).px > calm.px + 0.5, "a tailwind carries it farther")
        assertTrue(land(-8.0, 0.0).px < calm.px - 0.5, "a headwind holds it up")
        // Wind does not move a ball in vacuum mode (drag off).
        val w = world(vacuum); w.setWind(0.0, 10.0)
        launch(w, 30.0, 24.0, 1.8, 18.0, 0.0, 12.0)
        assertEquals(24.0, until(w, EventType.BallBounce).last().py, 1e-9)
    }

    @Test fun launchingTakesTheBallFromItsCarrier() {
        val w = world()
        val qb = player(w, 30.0, 24.0)
        w.setBallCarrier(qb)
        assertEquals(BallState.Carried, w.ball().state)
        launch(w, 30.0, 24.0, 1.8, 15.0, 0.0, 5.0)
        assertNull(w.ballCarrier)
        assertTrue(w.forwardProgressX.isNaN())
        assertEquals(BallState.Airborne, w.ball().state)
    }

    @Test fun rejectsBadLaunches() {
        val w = world()
        assertFailsWith<IllegalArgumentException> { launch(w, 30.0, 24.0, -0.1, 1.0, 0.0, 1.0) }
        assertFailsWith<IllegalArgumentException> { launch(w, Double.NaN, 24.0, 1.0, 1.0, 0.0, 1.0) }
        assertFailsWith<IllegalArgumentException> { launch(w, 30.0, 24.0, 1.0, Double.POSITIVE_INFINITY, 0.0, 1.0) }
        assertFailsWith<IllegalArgumentException> { w.setWind(Double.NaN, 0.0) }
    }

    // ------------------------------------------------------------ bounces and the loose ball

    @Test fun aBounceKeepsHalfTheVerticalSpeedLosesSomeGroundSpeedAndYawsWithinLimits() {
        val w = world(vacuum, seed = 4L)
        launch(w, 50.0, 24.0, 0.0, 10.0, 0.0, 10.0)
        val first = until(w, EventType.BallBounce).last()
        assertEquals(EventType.BallBounce, first.type)
        assertEquals(w.ballId.value, first.a)
        assertEquals(50.0 + 10.0 * 2 * 10.0 / g, first.px, 1e-9)
        assertEquals(DMath.hypot2(10.0, 10.0), first.value, 1e-9, "impact speed")
        val b = w.ball()
        assertEquals(BallState.Airborne, b.state, "e = 0.5 sends it back up at 5 m/s")
        // Friction: min(μ(1+e)|vz|, 0.4·vh) = min(7.5, 4) → 6 m/s along the ground, at a random yaw within ±35°.
        assertEquals(6.0, DMath.hypot2(b.vx, b.vy), 1e-9)
        val yaw = DMath.atan2(b.vy, b.vx)
        assertTrue(DMath.abs(yaw) <= BallConfig().bounceYawMax + 1e-12, "yaw $yaw")
        // Next bounce after 2·5/g s of air time.
        val t0 = w.tick
        until(w, EventType.BallBounce)
        assertEquals(2 * 5.0 / g * 60.0, (w.tick - t0).toDouble(), 1.0)
    }

    @Test fun bounceYawIsSeeded() {
        fun yaw(seed: Long): Double {
            val w = world(vacuum, seed)
            launch(w, 50.0, 24.0, 0.0, 10.0, 0.0, 10.0)
            until(w, EventType.BallBounce)
            return DMath.atan2(w.ball().vy, w.ball().vx)
        }
        assertEquals(yaw(9L), yaw(9L))
        assertNotEquals(yaw(9L), yaw(10L))
        // Over many seeds the yaws spread over most of ±35°.
        val ys = (1L..40L).map { yaw(it) }
        assertTrue(ys.max() > 0.4 && ys.min() < -0.4, "spread ${ys.min()}..${ys.max()}")
    }

    @Test fun surfaceScalesTheBounce() {
        fun rebound(s: SurfaceConfig): Double {
            val w = world(vacuum)
            w.setSurface(s)
            launch(w, 50.0, 24.0, 0.0, 0.0, 0.0, 10.0)
            until(w, EventType.BallBounce)
            val t0 = w.tick
            until(w, EventType.BallBounce)
            return (w.tick - t0) * WorldConfig.DT * g / 2 // rebound vertical speed
        }
        assertEquals(5.0, rebound(SurfaceConfig.ARTIFICIAL_TURF), 0.1)
        assertEquals(5.0 * 0.7, rebound(SurfaceConfig.WET), 0.1)
    }

    @Test fun lowBouncesHandTheBallToALooseBodyThatRollsToAStop() {
        val w = world(seed = 3L)
        launch(w, 50.0, 24.0, 0.0, 5.0, 0.0, 3.0)
        var t = 0
        while (w.ball().state == BallState.Airborne && t++ < 300) w.step()
        assertEquals(BallState.Loose, w.ball().state)
        val id = w.ballId
        val out = IntArray(8)
        assertTrue(id.value in out.copyOf(w.bodyIds(out)).toList(), "the loose ball is a body")
        val b = w.body(id)
        assertEquals(BodyType.Dynamic, b.type)
        assertEquals(Category.BALL, b.categoryBits)
        assertEquals(Material.BALL, b.material)
        assertEquals(BallConfig().mass, b.mass)
        assertEquals(BallConfig().radius, b.radius)
        assertTrue(b.bullet)
        // Rolls with friction: the speed drops by rollFriction·g per second.
        val v0 = DMath.hypot2(b.vx, b.vy)
        assertTrue(v0 > 0.5, "still rolling at $v0")
        w.step()
        val v1 = DMath.hypot2(w.body(id).vx, w.body(id).vy)
        assertEquals(BallConfig().rollFriction * g * WorldConfig.DT, v0 - v1, 1e-9)
        repeat(240) { w.step() }
        assertEquals(0.0, DMath.hypot2(w.body(id).vx, w.body(id).vy), 1e-12)
        assertEquals(w.body(id).x, w.ball().x); assertEquals(0.0, w.ball().z)
    }

    @Test fun aBallLaunchedAlongTheGroundIsLooseOnTheFirstTick() {
        val w = world()
        launch(w, 50.0, 24.0, 0.0, 6.0, 0.0, 0.0)
        w.step()
        assertEquals(BallState.Loose, w.ball().state)
        assertEquals(6.0, w.body(w.ballId).vx, 0.2)
    }

    @Test fun aLooseBallPassesThroughPlayersAsASensor() {
        val w = world()
        val p = player(w, 53.0, 24.0)
        launch(w, 50.0, 24.0, 0.0, 6.0, 0.0, 0.0)
        val ev = w.run(90)
        val begin = ev.first { it.type == EventType.ContactBegin }
        assertEquals(p.value, begin.a); assertEquals(w.ballId.value, begin.b)
        assertTrue(begin.sensor)
        assertEquals(0.0, w.body(p).vx, "the player is not pushed")
        assertTrue(w.body(w.ballId).x > 54.0, "the ball rolled through")
    }

    @Test fun aLooseBallGoingOutOfBoundsIsReported() {
        val w = world()
        launch(w, 50.0, 2.0, 0.0, 0.0, -8.0, 0.0)
        val ev = w.run(60)
        val out = ev.first { it.type == EventType.OutOfBounds }
        assertEquals(w.ballId.value, out.a); assertEquals(FieldLine.NEAR_SIDELINE, out.line)
    }

    @Test fun aLooseBallStopsAtTheRunoffWallAndThePostBase() {
        val w = world()
        launch(w, 30.0, -3.0, 0.0, 0.0, -30.0, 0.0)
        repeat(60) { w.step() }
        assertTrue(w.body(w.ballId).y > -f.runoffM, "stopped by the near runoff wall: y ${w.body(w.ballId).y}")
        // The gooseneck base behind the away end line, hit along the ground at 30 m/s.
        val w2 = world()
        launch(w2, f.lengthM - 8.0, cy, 0.0, 30.0, 0.0, 0.0)
        repeat(60) { w2.step() }
        assertTrue(w2.body(w2.ballId).x < f.lengthM + f.goalPostOffsetM, "x ${w2.body(w2.ballId).x}")
    }

    // ------------------------------------------------------------ possession

    @Test fun givingTheLooseBallToACarrierRemovesItsBodyAndItFollowsHim() {
        val w = world()
        val p = player(w, 55.0, 24.0)
        launch(w, 50.0, 24.0, 0.0, 6.0, 0.0, 0.0)
        repeat(10) { w.step() }
        assertEquals(BallState.Loose, w.ball().state)
        w.setBallCarrier(p)
        assertEquals(BallState.Carried, w.ball().state)
        val out = IntArray(8)
        assertEquals(listOf(p.value), out.copyOf(w.bodyIds(out)).toList())
        assertFailsWith<IllegalArgumentException> { w.body(w.ballId) }
        w.createBody(BodyDef(x = 70.0, y = 24.0)) // the ball's slot is not reused by a new body
        val pv = w.body(p)
        w.step()
        assertEquals(w.body(p).x, w.ball().x); assertEquals(w.body(p).y, w.ball().y)
        assertEquals(BallConfig().carryHeight, w.ball().z)
        w.setBallCarrier(null)
        assertEquals(BallState.Dead, w.ball().state)
        assertTrue(pv.x.isFinite())
    }

    @Test fun aCatchEndsTheFlight() {
        val w = world()
        val p = player(w, 60.0, 24.0)
        launch(w, 50.0, 24.0, 1.8, 15.0, 0.0, 3.0)
        repeat(20) { w.step() }
        w.setBallCarrier(p)
        assertEquals(BallState.Carried, w.ball().state)
        assertTrue(w.run(120).none { it.type == EventType.BallBounce })
    }

    @Test fun theBallBodyBelongsToTheEngine() {
        val w = world()
        val p = player(w, 55.0, 24.0)
        launch(w, 50.0, 24.0, 0.0, 1.0, 0.0, 0.0)
        w.step()
        val id = w.ballId
        assertFailsWith<IllegalArgumentException> { w.destroyBody(id) }
        assertFailsWith<IllegalArgumentException> { w.setStance(id, Stance.Down) }
        assertFailsWith<IllegalArgumentException> { w.setAthlete(id, AthleteParams(maxSpeed = 1.0, maxAccel = 1.0, maxDecel = 1.0, turnRateAtRest = 1.0, turnRateAtMax = 1.0)) }
        assertFailsWith<IllegalArgumentException> { w.createDistance(id, p, DistanceDef()) }
        assertFailsWith<IllegalArgumentException> { w.setBallCarrier(id) }
    }

    @Test fun maxBodiesStillCountsOnlyGameBodies() {
        val w = World(WorldConfig(maxBodies = 2), f, 1L)
        player(w, 10.0, 10.0); player(w, 20.0, 10.0)
        launch(w, 50.0, 24.0, 0.0, 1.0, 0.0, 0.0)
        w.step()
        assertEquals(BallState.Loose, w.ball().state)
        assertEquals(3, w.bodyCount)
        assertFailsWith<IllegalStateException> { player(w, 30.0, 10.0) }
    }

    // ------------------------------------------------------------ kicks

    /** Drag off, so the height at the plane is exact: 20 m/s along x reaches the plane after 1 s. */
    private fun kick(y: Double, zAtPlane: Double, towardAway: Boolean = true): List<Ev> {
        val w = world(vacuum)
        val vz = zAtPlane + 0.5 * g
        if (towardAway) launch(w, f.lengthM - 20.0, y, 0.0, 20.0, 0.0, vz, FlightMode.EndOverEnd)
        else launch(w, 20.0, y, 0.0, -20.0, 0.0, vz, FlightMode.EndOverEnd)
        return w.run(150).filter { it.type == EventType.KickResult }
    }

    @Test fun kickResultsAtTheEndLinePlane() {
        val good = kick(cy, 7.0).single()
        assertEquals(KickResult.Good, good.kick)
        assertEquals(f.lengthM, good.px, 1e-9); assertEquals(cy, good.py, 1e-9); assertEquals(7.0, good.pz, 1e-2)
        assertEquals(FieldLine.AWAY_END_LINE, good.line)
        // Under the bar, 1 m off center so it misses the gooseneck behind the end line.
        assertEquals(KickResult.Short, kick(cy + 1.0, 2.0).single().kick)
        // Left and right are the kicker's: toward +x, left is +y.
        assertEquals(KickResult.WideLeft, kick(cy + 4.0, 7.0).single().kick)
        assertEquals(KickResult.WideRight, kick(cy - 4.0, 7.0).single().kick)
        assertEquals(KickResult.WideRight, kick(cy + 4.0, 7.0, towardAway = false).single().kick)
        assertEquals(KickResult.WideLeft, kick(cy - 4.0, 7.0, towardAway = false).single().kick)
        val home = kick(cy, 7.0, towardAway = false).single()
        assertEquals(KickResult.Good, home.kick); assertEquals(FieldLine.HOME_END_LINE, home.line)
        // High above the uprights but between their extended lines is still good.
        assertEquals(KickResult.Good, kick(cy + 2.0, 30.0).single().kick)
    }

    @Test fun hittingAnUprightOrTheCrossbarIsHitPostAndTheBallBouncesOff() {
        for (towardAway in listOf(true, false)) {
            val upright = kick(cy + f.uprightSpacingM / 2, 7.0, towardAway)
            assertEquals(KickResult.HitPost, upright.first().kick, "upright, toward away $towardAway")
            val bar = kick(cy, f.crossbarHeightM, towardAway)
            assertEquals(KickResult.HitPost, bar.first().kick, "crossbar")
        }
        // A dead-center hit on the upright sends the ball back toward the field.
        val w = world(vacuum)
        launch(w, f.lengthM - 20.0, cy + f.uprightSpacingM / 2, 0.0, 20.0, 0.0, 7.0 + 0.5 * g)
        until(w, EventType.KickResult)
        assertTrue(w.ball().vx < 0.0, "rebounds: vx ${w.ball().vx}")
        assertEquals(-20.0 * 0.6, w.ball().vx, 0.5, "with e = 0.6")
    }

    @Test fun aKickThatLandsShortOfThePlaneReportsNoKickResult() {
        val w = world(vacuum)
        launch(w, f.lengthM - 40.0, cy, 0.0, 10.0, 0.0, 10.0)
        assertTrue(w.run(400).none { it.type == EventType.KickResult })
    }

    // ------------------------------------------------------------ catch windows

    @Test fun aPlayerWithReachGetsOneCatchOpportunity() {
        val w = world(vacuum)
        val r = player(w, 50.0, 24.0)
        w.setReach(r, 1.0, 2.5)
        launch(w, 40.0, 24.0, 2.0, 20.0, 0.0, 2.0)
        val ev = w.run(90).filter { it.type == EventType.CatchOpportunity }
        val c = ev.single()
        assertEquals(r.value, c.a)
        assertTrue(c.pz in 0.3..2.5, "z ${c.pz}")
        assertTrue(DMath.hypot2(c.px - 50.0, c.py - 24.0) <= 1.0)
        val vzAt = 2.0 - g * (c.px - 40.0) / 20.0
        assertEquals(DMath.sqrt(400.0 + vzAt * vzAt), c.value, 0.2, "ball speed")
        assertEquals(0.0, c.angleA, 1e-9, "arrival angle: the ball travels the way he faces")
        assertEquals(DMath.atan2(-vzAt, 20.0), c.angleB, 0.02, "descent angle")
    }

    @Test fun catchOpportunityArrivalAngleIsRelativeToHeading() {
        val w = world(vacuum)
        val r = player(w, 50.0, 24.0, angle = DMath.PI) // facing the passer
        w.setReach(r, 1.0, 2.5)
        launch(w, 40.0, 24.0, 2.0, 20.0, 0.0, 2.0)
        val c = w.run(90).single { it.type == EventType.CatchOpportunity }
        assertEquals(DMath.PI, DMath.abs(c.angleA), 1e-9)
    }

    @Test fun noCatchOpportunityOutOfReachOrWithoutReach() {
        val w = world(vacuum)
        val short = player(w, 50.0, 24.0)
        w.setReach(short, 1.0, 1.0) // ball goes by at ~1.8 m
        player(w, 48.5, 24.0) // right under the ball's path, but no reach set
        launch(w, 40.0, 24.0, 2.0, 20.0, 0.0, 2.0)
        assertTrue(w.run(90).none { it.type == EventType.CatchOpportunity })
        assertFailsWith<IllegalArgumentException> { w.setReach(short, -1.0, 2.0) }
        assertFailsWith<IllegalArgumentException> { w.setReach(short, 1.0, Double.NaN) }
    }

    // ------------------------------------------------------------ deflect

    @Test fun deflectChangesTheAirborneBallsVelocity() {
        val w = world(vacuum)
        launch(w, 40.0, 24.0, 3.0, 15.0, 0.0, 6.0)
        w.step()
        val m = BallConfig().mass
        val vx = w.ball().vx; val vz = w.ball().vz
        w.deflect(w.ballId, Vec2(-0.41 * 5.0, 0.41 * 2.0), 0.41 * 1.0)
        assertEquals(vx - 5.0 * 0.41 / m, w.ball().vx, 1e-12)
        assertEquals(2.0 * 0.41 / m, w.ball().vy, 1e-12)
        assertEquals(vz + 0.41 / m, w.ball().vz, 1e-12)
    }

    @Test fun deflectingTheLooseBallUpwardPopsItIntoTheAir() {
        val w = world()
        launch(w, 40.0, 24.0, 0.0, 3.0, 0.0, 0.0)
        w.step()
        assertEquals(BallState.Loose, w.ball().state)
        w.deflect(w.ballId, Vec2(1.0, 0.0), 2.0)
        assertEquals(BallState.Airborne, w.ball().state)
        assertTrue(w.ball().vz > 4.0)
        val out = IntArray(4)
        assertEquals(0, w.bodyIds(out), "no ball body while airborne")
        assertTrue(w.run(120).any { it.type == EventType.BallBounce })
    }

    @Test fun deflectWorksOnOtherDynamicBodiesAndRejectsADeadBall() {
        val w = world()
        val p = player(w, 40.0, 24.0)
        w.deflect(p, Vec2(100.0, 0.0))
        assertEquals(1.0, w.body(p).vx, 1e-12)
        assertFailsWith<IllegalArgumentException> { w.deflect(p, Vec2(1.0, 0.0), 1.0) }
        assertFailsWith<IllegalStateException> { w.deflect(w.ballId, Vec2(1.0, 0.0)) }
        val wall = w.createBody(BodyDef(type = BodyType.Static, x = 1.0, y = 1.0))
        assertFailsWith<IllegalArgumentException> { w.deflect(wall, Vec2(1.0, 0.0)) }
    }

    // ------------------------------------------------------------ launch solvers

    @Test fun apexSolverReachesTheArcHeightAndTheTarget() {
        for (mode in FlightMode.entries) {
            val w = world()
            w.setWind(3.0, -2.0)
            val from = Vec3(30.0, 24.0, 1.8)
            val target = Vec3(65.0, 32.0, 0.0)
            val v = w.launchVelocityFor(from, target, 14.0, mode, Vec3())
            w.launchBall(from, v, mode)
            var apex = 0.0
            var bounce: Ev? = null
            while (bounce == null) { w.step(); apex = DMath.max(apex, w.ball().z); bounce = w.drain().firstOrNull { it.type == EventType.BallBounce } }
            assertEquals(14.0, apex, 0.05, "$mode apex")
            assertEquals(0.0, DMath.hypot2(bounce.px - 65.0, bounce.py - 32.0), 0.01, "$mode landing")
        }
        val w = world()
        assertFailsWith<IllegalArgumentException> { w.launchVelocityFor(Vec3(30.0, 24.0, 3.0), Vec3(50.0, 24.0, 0.0), 2.0, FlightMode.Spiral, Vec3()) }
    }

    @Test fun speedSolverFindsTheLowAndHighArcs() {
        val w = world()
        val from = Vec3(30.0, 24.0, 1.8)
        val target = Vec3(70.0, 20.0, 1.5)
        val low = Vec3(); val high = Vec3()
        assertTrue(w.launchVelocityFor(from, target, 26.0, false, FlightMode.Spiral, low))
        assertTrue(w.launchVelocityFor(from, target, 26.0, true, FlightMode.Spiral, high))
        assertTrue(high.z > low.z + 5.0, "high arc ${high.z} vs low ${low.z}")
        val tLow = w.timeToArrive(from, low, FlightMode.Spiral, target)
        val tHigh = w.timeToArrive(from, high, FlightMode.Spiral, target)
        assertTrue(tHigh > tLow + 1.0, "high arc takes longer: $tHigh vs $tLow")
        assertFalse(w.launchVelocityFor(from, Vec3(29.0 + 150.0, 24.0, 0.0), 20.0, false, FlightMode.Spiral, Vec3()))
    }

    @Test fun timeToArriveMatchesTheSimulatedFlight() {
        val w = world()
        w.setWind(-4.0, 0.0)
        val from = Vec3(30.0, 24.0, 1.8)
        val target = Vec3(60.0, 24.0, 1.5)
        val v = w.launchVelocityFor(from, target, 10.0, FlightMode.Spiral, Vec3())
        val t = w.timeToArrive(from, v, FlightMode.Spiral, target)
        w.launchBall(from, v, FlightMode.Spiral)
        var ticks = 0
        while (w.ball().x < 60.0) { w.step(); ticks++ }
        assertEquals(ticks * WorldConfig.DT, t, WorldConfig.DT)
        // A ball that lands first never arrives.
        assertTrue(w.timeToArrive(from, Vec3(5.0, 0.0, 2.0), FlightMode.Spiral, target).isNaN())
    }

    @Test fun predictLandingIsExactlyWhereTheWorldLandsTheBall() {
        val w = world()
        w.setWind(2.0, 5.0)
        val from = Vec3(30.0, 24.0, 1.8)
        val v = Vec3(18.0, 3.0, 11.0)
        val land = Vec3()
        val t = w.predictLanding(from, v, FlightMode.Wobble, land)
        w.launchBall(from, v, FlightMode.Wobble)
        val b = until(w, EventType.BallBounce).last()
        assertEquals(land.x, b.px); assertEquals(land.y, b.py); assertEquals(0.0, land.z)
        assertEquals(b.tick.toDouble(), t * 60.0, 1.0)
    }

    // ------------------------------------------------------------ CCD

    @Test fun bulletBodiesDoNotTunnelThroughThinStaticsFromEitherSide() {
        for (dir in listOf(1.0, -1.0)) {
            val w = world()
            w.createBody(BodyDef(type = BodyType.Static, x = 55.0, y = 24.0, shape = ShapeDef.Segment(0.0, -4.0, 0.0, 4.0), material = Material.STRUCTURE, categoryBits = Category.STATIC))
            val b = w.createBody(BodyDef(x = 55.0 - 5.0 * dir, y = 24.0, vx = 120.0 * dir, mass = 1.0, bullet = true, shape = ShapeDef.Circle(0.1)))
            repeat(20) { w.step() }
            assertTrue(dir * (w.body(b).x - 55.0) < 0.0, "dir $dir: x ${w.body(b).x}")
        }
    }

    @Test fun sweptSphereAgainstACapsuleInBothDirections() {
        val s = Sweep3()
        // Capsule along z from (0,0,0) to (0,0,10), radius 0.2. A ray along +x at z = 5 from x = −2 to 2.
        assertTrue(s.capsule(-2.0, 0.0, 5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
        assertEquals(1.8 / 4.0, s.t, 1e-12); assertEquals(-1.0, s.nx, 1e-12)
        assertTrue(s.capsule(2.0, 0.0, 5.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
        assertEquals(1.8 / 4.0, s.t, 1e-12); assertEquals(1.0, s.nx, 1e-12)
        // The capsule's end is a sphere: straight down onto the top.
        assertTrue(s.capsule(0.0, 0.0, 12.0, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
        assertEquals(1.8 / 4.0, s.t, 1e-12); assertEquals(1.0, s.nz, 1e-12)
        // Swapping the capsule's ends changes nothing.
        assertTrue(s.capsule(-2.0, 0.0, 5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.2))
        assertEquals(1.8 / 4.0, s.t, 1e-12)
        // Misses: passing beside it, stopping short, and moving away.
        assertFalse(s.capsule(-2.0, 0.3, 5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
        assertFalse(s.capsule(-2.0, 0.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
        assertFalse(s.capsule(-2.0, 0.0, 5.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.2))
    }

    // ------------------------------------------------------------ recorder, determinism, allocation

    @Test fun recorderSeesEveryBallCommand() {
        val w = world()
        val p = player(w, 40.0, 24.0)
        val log = ArrayList<String>()
        w.recorder = object : CommandRecorder {
            override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) {}
            override fun onDestroyBody(tick: Long, id: BodyId) {}
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
            override fun onLaunchBall(tick: Long, from: Vec3, velocity: Vec3, mode: FlightMode) { log += "launch:${from.x},${velocity.z},$mode@$tick" }
            override fun onDeflect(tick: Long, id: BodyId, impulseX: Double, impulseY: Double, impulseZ: Double) { log += "deflect:${id.value}:$impulseX,$impulseY,$impulseZ@$tick" }
            override fun onSetWind(tick: Long, x: Double, y: Double) { log += "wind:$x,$y@$tick" }
            override fun onSetReach(tick: Long, id: BodyId, reachRadius: Double, reachHeight: Double) { log += "reach:${id.value}:$reachRadius,$reachHeight@$tick" }
        }
        w.setWind(1.0, 2.0)
        w.setReach(p, 1.1, 2.6)
        w.step()
        launch(w, 30.0, 24.0, 1.0, 10.0, 0.0, 5.0, FlightMode.Wobble)
        w.deflect(w.ballId, Vec2(0.5, 0.0), 0.25)
        assertEquals(listOf("wind:1.0,2.0@0", "reach:${p.value}:1.1,2.6@0", "launch:30.0,5.0,Wobble@1", "deflect:${w.ballId.value}:0.5,0.0,0.25@1"), log)
    }

    private fun ballWorld(seed: Long): World {
        val w = World(WorldConfig(), f, seed)
        w.setWind(2.0, -3.0)
        // The receiver runs onto the spot (about 2.45 s of flight away); a defender stands off to the side.
        val r = w.createBody(BodyDef(x = 56.2, y = 22.0, vx = 4.0, shape = ShapeDef.Circle(0.33), categoryBits = Category.HOME))
        w.setReach(r, 1.2, 2.8)
        w.createBody(BodyDef(x = 67.0, y = 20.5, shape = ShapeDef.Circle(0.33), categoryBits = Category.AWAY))
        val from = Vec3(40.0, 24.0, 1.9)
        val v = Vec3()
        w.launchVelocityFor(from, Vec3(66.0, 22.0, 1.4), 9.0, FlightMode.Wobble, v)
        w.launchBall(from, v, FlightMode.Wobble)
        return w
    }

    @Test fun ballFlightIsDeterministic() {
        fun run(seed: Long): Pair<List<Long>, List<Ev>> {
            val w = ballWorld(seed)
            val h = ArrayList<Long>(); val e = ArrayList<Ev>()
            repeat(400) { w.step(); h += w.stateHash(); e += w.drain() }
            return h to e
        }
        val a = run(5L)
        assertEquals(a, run(5L))
        assertTrue(a.second.any { it.type == EventType.CatchOpportunity } && a.second.count { it.type == EventType.BallBounce } >= 2)
        assertNotEquals(a.first, run(6L).first, "a different seed bounces differently")
    }

    @Test fun ballFlightDoesNotAllocate() {
        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        var sink = 0.0
        val from = Vec3(); val v = Vec3(); val tgt = Vec3(); val imp = Vec2()
        fun tick(w: World) {
            w.step()
            val e = w.events()
            while (e.next()) {
                sink += e.tick + e.bodyA.value + e.pointX + e.pointZ + e.speed + e.arrivalAngle + e.descentAngle
                if (e.type == EventType.KickResult) sink += e.kickResult.ordinal
            }
            val b = w.ball()
            sink += b.x + b.y + b.z + b.vx + b.state.ordinal + b.flightMode.ordinal + w.stateHash()
            if (b.state == BallState.Loose) sink += w.body(w.ballId).vx
        }
        /** Round kinds: a pass with a catch window and bounces, a field goal off the upright, a loose-ball batted up. */
        fun setup(round: Int): World {
            return when (round % 3) {
                0 -> ballWorld(round.toLong())
                1 -> World(WorldConfig(), f, round.toLong()).also {
                    from.set(f.lengthM - 30.0, cy, 0.0)
                    it.launchVelocityFor(from, tgt.set(f.lengthM, cy + f.uprightSpacingM / 2, 6.0), 26.0, false, FlightMode.EndOverEnd, v)
                    it.launchBall(from, v, FlightMode.EndOverEnd)
                }
                else -> World(WorldConfig(), f, round.toLong()).also {
                    it.launchBall(from.set(50.0, 24.0, 0.0), v.set(8.0, 1.0, 0.0), FlightMode.Spiral)
                }
            }
        }
        fun play(w: World, round: Int) {
            repeat(200) {
                tick(w)
                // Bat the rolling ball back up once: deflect is a command (between steps), but the flight it starts is in step().
                if (round % 3 == 2 && it == 30) w.deflect(w.ballId, imp.set(0.3, 0.0), 1.5)
            }
        }
        for (round in 0 until 12) { val w = setup(round); play(w, round) }
        val tid = Thread.currentThread().id
        // The same 1200 steps up to three times. The JVM sometimes allocates a few hundred bytes once (linking or
        // deoptimizing a path the warm-up compiled differently); an allocation in the engine repeats every attempt.
        val attempts = ArrayList<Long>()
        while (attempts.size < 3 && (attempts.isEmpty() || attempts.last() != 0L)) {
            var allocated = 0L
            for (round in 0 until 6) {
                val w = setup(round) // creation and launch stay outside the measured window
                val before = mx.getThreadAllocatedBytes(tid)
                play(w, round)
                allocated += mx.getThreadAllocatedBytes(tid) - before
            }
            attempts += allocated
        }
        assertEquals(0L, attempts.last(), "bytes allocated by 1200 steps of ball flight, per attempt $attempts (sink $sink)")
    }
}
