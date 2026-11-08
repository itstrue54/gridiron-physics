package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Vec2
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class EventsTest {
    private val f = FieldConfig.PRO

    private fun world(config: WorldConfig = WorldConfig()) = World(config, f, 1L)

    private fun player(
        w: World, x: Double, y: Double, vx: Double = 0.0, vy: Double = 0.0, angle: Double = 0.0,
        mass: Double = 100.0, category: Int = Category.HOME, mask: Int = Category.ALL, sensor: Boolean = false,
        type: BodyType = BodyType.Dynamic,
    ): BodyId = w.createBody(
        BodyDef(
            type = type, x = x, y = y, vx = vx, vy = vy, angle = angle, mass = mass,
            shape = ShapeDef.Circle(0.34, sensor), categoryBits = category, maskBits = mask,
        ),
    )

    private val brakeHard = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 12.0, turnRateAtRest = 10.0, turnRateAtMax = 2.0, traction = 0.9)

    /** A runner braking from 9 m/s with maxDecel above traction: one Slip per tick for about a second. */
    private fun braker(w: World, y: Double = 24.0): BodyId {
        val p = w.createBody(BodyDef(x = 30.0, y = y, vx = 9.0, mass = 95.0, shape = ShapeDef.Circle(0.33)))
        w.setAthlete(p, brakeHard)
        w.setIntent(p, LocomotionIntent.velocity(0.0, 0.0))
        return p
    }

    // ------------------------------------------------------------ the ring buffer

    @Test fun readerStartsEmptyAndHasNoCurrentEvent() {
        val w = world()
        val r = w.events()
        assertEquals(0, r.pending)
        assertFalse(r.next())
        assertFailsWith<IllegalStateException> { r.type }
        assertTrue(w.events() === r, "events() always returns the same reader")
    }

    @Test fun eventsStayUntilReadAndCarryTheirTick() {
        val w = world()
        braker(w)
        w.advance(3 * WorldConfig.DT) // three ticks in one frame
        assertEquals(3L, w.tick)
        val evs = w.drain()
        assertEquals(listOf(1L, 2L, 3L), evs.map { it.tick })
        assertTrue(evs.all { it.type == EventType.Slip })
        assertEquals(0, w.events().pending)
    }

    @Test fun fullBufferDropsTheOldestEvents() {
        val w = world(WorldConfig(eventCapacity = 4))
        braker(w)
        repeat(10) { w.step() }
        val r = w.events()
        assertEquals(4, r.pending)
        assertEquals(6L, r.lostCount)
        assertEquals(listOf(7L, 8L, 9L, 10L), w.drain().map { it.tick })
    }

    @Test fun clearDropsUnreadEvents() {
        val w = world()
        braker(w)
        repeat(3) { w.step() }
        w.events().clear()
        assertEquals(0, w.events().pending)
        assertFalse(w.events().next())
    }

    // ------------------------------------------------------------ Slip (replaces M3's temporary accessors)

    @Test fun slipNamesTheBodyAndTheExcess() {
        val w = world()
        braker(w, y = 10.0)
        val p = braker(w, y = 30.0)
        w.step()
        val slips = w.drain().filter { it.type == EventType.Slip }
        assertEquals(2, slips.size, "one per braking body, ascending id")
        assertEquals(p.value, slips[1].a)
        assertEquals(12.0 - 0.9 * 9.81, slips[1].value, 1e-9)
    }

    // ------------------------------------------------------------ contact begin / end

    @Test fun contactBeginsOnceAndEndsOnce() {
        val w = world()
        val a = player(w, 10.0, 20.0, vx = 6.0)
        val b = player(w, 12.0, 20.0)
        val evs = w.run(120)
        val begins = evs.filter { it.type == EventType.ContactBegin }
        val ends = evs.filter { it.type == EventType.ContactEnd }
        assertEquals(1, begins.size, "one begin: $begins")
        assertEquals(1, ends.size, "one end: $ends")
        val bg = begins[0]
        assertEquals(a.value, bg.a); assertEquals(b.value, bg.b)
        assertEquals(ContactView.NO_WALL, bg.wall)
        assertFalse(bg.sensor)
        assertEquals(1.0, bg.nx, 1e-9, "normal points from A to B")
        assertTrue(bg.px > 10.0 && bg.px < 12.0 + 0.34, "point between the bodies: ${bg.px}")
        assertEquals(20.0, bg.py, 1e-9)
        assertTrue(ends[0].tick > bg.tick)
        assertEquals(a.value, ends[0].a); assertEquals(b.value, ends[0].b)
    }

    @Test fun beginArrivesOnTheImpactTick() {
        // At 6 m/s the speculative contact closes the 1.32 m gap exactly; impact is on the tick the gap runs out.
        val w = world()
        player(w, 10.0, 20.0, vx = 6.0)
        player(w, 12.0, 20.0)
        val gap = 2.0 - 0.68
        val impactTick = DMath.floor(gap / (6.0 * WorldConfig.DT)).toLong() + 1
        val begin = w.run(60).first { it.type == EventType.ContactBegin }
        assertEquals(impactTick, begin.tick)
    }

    @Test fun wallContactNamesTheWall() {
        val w = world()
        player(w, 30.0, -5.0, vy = -6.0)
        val begin = w.run(60).first { it.type == EventType.ContactBegin }
        assertEquals(ContactView.WALL_NEAR, begin.wall)
        assertEquals(-1, begin.b)
    }

    @Test fun bodyBThrowsForWallsAndLineEvents() {
        val w = world()
        player(w, 30.0, -5.0, vy = -6.0)
        val r = w.events()
        var sawWall = false
        repeat(60) {
            w.step()
            while (r.next()) if (r.type == EventType.ContactBegin) { assertFailsWith<IllegalStateException> { r.bodyB }; sawWall = true }
        }
        assertTrue(sawWall)
        player(w, f.awayGoalLineX - 1.0, 20.0, vx = 6.0)
        var sawLine = false
        repeat(30) {
            w.step()
            while (r.next()) if (r.type == EventType.SensorBegin) { assertFailsWith<IllegalStateException> { r.bodyB }; sawLine = true }
        }
        assertTrue(sawLine)
    }

    @Test fun pairsThatFallAsleepTouchingReportNothingUntilTheySeparate() {
        val w = world()
        val a = player(w, 20.0, 20.0)
        val b = player(w, 20.67, 20.0) // 1 cm overlap
        val first = w.run(240)
        assertEquals(1, first.count { it.type == EventType.ContactBegin })
        assertEquals(0, first.count { it.type == EventType.ContactEnd }, "no end while resting or asleep")
        assertFalse(w.body(a).awake); assertFalse(w.body(b).awake)
        w.setStance(a, Stance.Upright) // wakes A without moving anything
        val woke = w.run(5)
        assertEquals(0, woke.count { it.type == EventType.ContactBegin || it.type == EventType.ContactEnd }, "waking is not a new contact: $woke")
        w.destroyBody(b)
        val gone = w.run(1)
        val end = gone.single { it.type == EventType.ContactEnd }
        assertEquals(a.value, end.a); assertEquals(b.value, end.b)
    }

    @Test fun sensorShapesReportContactsAsSensor() {
        val w = world()
        player(w, 10.0, 20.0, vx = 6.0)
        player(w, 12.0, 20.0, sensor = true)
        val evs = w.run(60)
        val begin = evs.single { it.type == EventType.ContactBegin }
        assertTrue(begin.sensor)
        assertTrue(evs.none { it.type == EventType.Hit }, "a sensor never hits")
    }

    // ------------------------------------------------------------ Hit

    private fun headOn(moverFirst: Boolean): Pair<Ev, Pair<BodyId, BodyId>> {
        val w = world()
        val mover: BodyId
        val target: BodyId
        if (moverFirst) {
            mover = player(w, 10.0, 20.0, vx = 6.0)
            target = player(w, 12.0, 20.0, mass = 120.0)
        } else {
            target = player(w, 12.0, 20.0, mass = 120.0)
            mover = player(w, 10.0, 20.0, vx = 6.0)
        }
        val hit = w.run(60).single { it.type == EventType.Hit }
        return hit to (mover to target)
    }

    @Test fun hitReportsLowerIdFirstInBothArgumentOrders() {
        for (moverFirst in listOf(true, false)) {
            val (hit, ids) = headOn(moverFirst)
            val (mover, target) = ids
            val lo = minOf(mover.value, target.value)
            val hi = maxOf(mover.value, target.value)
            assertEquals(lo, hit.a); assertEquals(hi, hit.b)
            // The normal points from A to B whichever body moved.
            val towardB = if (hit.a == mover.value) 1.0 else -1.0
            assertEquals(towardB, hit.nx, 1e-9, "moverFirst=$moverFirst")
            assertEquals(0.0, hit.ny, 1e-9)
            assertEquals(6.0, hit.closingSpeed, 1e-9, "closing speed before impact")
            // Momentum before impact: 100 kg at 6 m/s, 120 kg at rest.
            val (mMover, mTarget) = if (hit.a == mover.value) (hit.momAX to hit.momBX) else (hit.momBX to hit.momAX)
            assertEquals(600.0, mMover, 1e-9); assertEquals(0.0, mTarget, 1e-9)
            assertEquals(0.0, hit.momAY, 1e-12); assertEquals(0.0, hit.momBY, 1e-12)
            // First-tick impulse: reduced mass × (1 + e) × closing speed, e = 0.02 for player–player.
            val mu = 100.0 * 120.0 / 220.0
            assertEquals(mu * 1.02 * 6.0, hit.normalImpulse, 0.01 * mu * 6.0, "moverFirst=$moverFirst")
            // Both face +x: the mover is hit dead ahead, the target from behind.
            val (angMover, angTarget) = if (hit.a == mover.value) (hit.angleA to hit.angleB) else (hit.angleB to hit.angleA)
            assertEquals(0.0, angMover, 1e-9)
            assertEquals(DMath.PI, angTarget, 1e-9, "from behind is +π, never −π")
            assertTrue(hit.px > 10.0 && hit.px < 12.0)
        }
    }

    @Test fun hitAngleIsRelativeToEachHeading() {
        val w = world()
        val mover = player(w, 10.0, 20.0, vx = 6.0, angle = 0.0)
        val target = player(w, 12.0, 20.0, angle = DMath.PI / 2) // facing +y, hit on his right side
        val hit = w.run(60).single { it.type == EventType.Hit }
        assertEquals(mover.value, hit.a); assertEquals(target.value, hit.b)
        assertEquals(DMath.PI / 2, hit.angleB, 1e-9)
    }

    @Test fun onlyPlayerPairsHit() {
        val w = world()
        player(w, 10.0, 20.0, vx = 6.0)
        player(w, 12.0, 20.0, category = Category.OFFICIAL)
        player(w, 10.0, 30.0, vx = 6.0)
        w.createBody(BodyDef(type = BodyType.Static, x = 12.0, y = 30.0, shape = ShapeDef.Circle(0.5), categoryBits = Category.STATIC, material = Material.STRUCTURE))
        val evs = w.run(60)
        assertEquals(2, evs.count { it.type == EventType.ContactBegin })
        assertEquals(0, evs.count { it.type == EventType.Hit })
    }

    @Test fun hitFollowsItsContactBegin() {
        val evs = world().let { w -> player(w, 10.0, 20.0, vx = 6.0); player(w, 12.0, 20.0); w.run(60) }
        val i = evs.indexOfFirst { it.type == EventType.ContactBegin }
        assertEquals(EventType.Hit, evs[i + 1].type)
        assertEquals(evs[i].tick, evs[i + 1].tick)
    }

    // ------------------------------------------------------------ Trip

    private fun tripSetup(runnerFirst: Boolean, speed: Double, startX: Double = 46.0): Triple<List<Ev>, BodyId, BodyId> {
        val w = world()
        val runner: BodyId
        val down: BodyId
        if (runnerFirst) {
            runner = player(w, startX, 20.0, vx = speed)
            down = player(w, 50.0, 20.0)
        } else {
            down = player(w, 50.0, 20.0)
            runner = player(w, startX, 20.0, vx = speed)
        }
        w.setStance(down, Stance.Down)
        return Triple(w.run(240), runner, down)
    }

    @Test fun movingPlayerTripsOverADownBodyInBothArgumentOrders() {
        for (runnerFirst in listOf(true, false)) {
            val (evs, runner, down) = tripSetup(runnerFirst, 5.0)
            val trip = evs.single { it.type == EventType.Trip }
            assertEquals(runner.value, trip.a, "A is the mover")
            assertEquals(down.value, trip.b, "B is the Down body")
            assertEquals(5.0, trip.value, 1e-9, "speed before contact")
            assertEquals(1.0, trip.nx, 1e-9, "normal from mover to Down body (runnerFirst=$runnerFirst)")
            assertTrue(evs.any { it.type == EventType.Hit }, "it is a player–player contact too")
        }
    }

    @Test fun slowPlayersDoNotTrip() {
        val (evs, _, _) = tripSetup(true, 0.5, startX = 48.3) // 0.48 m from the capsule's end
        assertTrue(evs.any { it.type == EventType.ContactBegin })
        assertTrue(evs.none { it.type == EventType.Trip })
    }

    // ------------------------------------------------------------ field lines

    private fun lineEvents(evs: List<Ev>, id: BodyId) = evs.filter {
        it.a == id.value && it.type in setOf(EventType.SensorBegin, EventType.SensorEnd, EventType.OutOfBounds, EventType.GoalLineCrossed)
    }

    @Test fun crossingAGoalLineInBothDirections() {
        for ((line, gx) in listOf(FieldLine.AWAY_GOAL_LINE to f.awayGoalLineX, FieldLine.HOME_GOAL_LINE to f.homeGoalLineX)) {
            for (dir in listOf(1, -1)) {
                val w = world()
                val p = player(w, gx - dir * 2.0, 20.0, vx = dir * 6.0)
                val evs = lineEvents(w.run(60), p)
                assertEquals(listOf(EventType.SensorBegin, EventType.GoalLineCrossed, EventType.SensorEnd), evs.map { it.type }, "line $line dir $dir")
                assertTrue(evs.all { it.line == line })
                assertTrue(evs[0].tick < evs[1].tick && evs[1].tick < evs[2].tick)
                val cross = evs[1]
                assertEquals(dir, cross.direction)
                assertEquals(gx, cross.px, 1e-9)
                assertEquals(20.0, cross.py, 1e-9)
            }
        }
    }

    @Test fun goalLineExtendedBeyondTheSidelineIsNotAGoalLine() {
        val w = world()
        val p = player(w, f.awayGoalLineX - 2.0, -3.0, vx = 6.0)
        assertTrue(lineEvents(w.run(60), p).isEmpty())
    }

    @Test fun touchingTheSidelineIsOutOfBoundsOnce() {
        val w = world()
        val p = w.createBody(BodyDef(x = 50.0, y = 2.0, mass = 95.0, shape = ShapeDef.Circle(0.33)))
        w.setAthlete(p, AthleteParams())
        w.setIntent(p, LocomotionIntent.point(50.0, -2.0))
        val out1 = lineEvents(w.run(180), p)
        assertEquals(listOf(EventType.SensorBegin, EventType.OutOfBounds, EventType.SensorEnd), out1.map { it.type })
        assertEquals(out1[0].tick, out1[1].tick, "touching the line is out")
        assertTrue(out1.all { it.line == FieldLine.NEAR_SIDELINE })
        // Back in, then out again: a second OutOfBounds.
        w.setIntent(p, LocomotionIntent.point(50.0, 3.0))
        val back = lineEvents(w.run(180), p)
        assertEquals(listOf(EventType.SensorBegin, EventType.SensorEnd), back.map { it.type }, "coming back in is not an event")
        w.setIntent(p, LocomotionIntent.point(50.0, -2.0))
        assertEquals(1, lineEvents(w.run(180), p).count { it.type == EventType.OutOfBounds })
    }

    @Test fun crossingTheEndLineIsOutOfBounds() {
        val w = world()
        val p = player(w, f.lengthM - 1.0, 20.0, vx = 4.0)
        val evs = lineEvents(w.run(60), p)
        val oob = evs.single { it.type == EventType.OutOfBounds }
        assertEquals(FieldLine.AWAY_END_LINE, oob.line)
    }

    @Test fun bodiesCreatedOutOfBoundsOrOnALineReportNothing() {
        val w = world()
        val out = player(w, 50.0, -3.0)
        val on = player(w, 50.0, 0.1)
        val evs = w.run(30)
        assertTrue(lineEvents(evs, out).isEmpty()); assertTrue(lineEvents(evs, on).isEmpty())
    }

    @Test fun bodiesWithoutTheLineSensorMaskAreNotWatched() {
        val w = world()
        val p = player(w, f.awayGoalLineX - 2.0, 1.0, vx = 6.0, vy = -2.0, mask = Category.ALL xor Category.LINE_SENSOR)
        assertTrue(lineEvents(w.run(60), p).isEmpty())
    }

    @Test fun staticBodiesAreNotWatched() {
        val w = world()
        w.createBody(BodyDef(type = BodyType.Static, x = f.homeGoalLineX, y = 20.0, shape = ShapeDef.Circle(0.5), categoryBits = Category.STATIC))
        assertTrue(w.run(10).isEmpty())
    }

    // ------------------------------------------------------------ determinism and allocation

    private fun busyWorld(): World {
        val w = world()
        val ap = AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)
        for (k in 0 until 6) {
            for (side in 0 until 2) {
                val id = w.createBody(
                    BodyDef(
                        x = if (side == 0) f.awayGoalLineX - 4.0 else f.awayGoalLineX + 2.0, y = 0.6 + k * 0.8 + side * 0.3,
                        mass = 95.0, shape = ShapeDef.Circle(0.33), categoryBits = if (side == 0) Category.HOME else Category.AWAY,
                    ),
                )
                w.setAthlete(id, ap)
            }
        }
        return w
    }

    /** Players running back and forth across a goal line and the near sideline, meeting in the middle. */
    private fun drive(w: World, round: Int, ids: IntArray, n: Int) {
        for (i in 0 until n) {
            val home = i % 2 == 0
            val x = if ((round % 2 == 0) == home) f.awayGoalLineX + 3.0 else f.awayGoalLineX - 3.0
            val y = if (round % 3 == 0) -1.0 else 3.0
            w.setIntent(BodyId(ids[i]), LocomotionIntent.point(x, y))
        }
    }

    @Test fun eventStreamIsDeterministic() {
        fun run(): List<Ev> {
            val w = busyWorld()
            val ids = IntArray(64); val n = w.bodyIds(ids)
            val all = ArrayList<Ev>()
            for (round in 0 until 6) { drive(w, round, ids, n); all += w.run(120) }
            return all
        }
        val a = run()
        assertTrue(a.map { it.type }.toSet().containsAll(listOf(EventType.Hit, EventType.OutOfBounds, EventType.GoalLineCrossed, EventType.SensorBegin, EventType.ContactEnd)), "busy world covers the event kinds: ${a.map { it.type }.toSet()}")
        assertEquals(a, run())
    }

    @Test fun stepWithEventsAndQueriesDoesNotAllocate() {
        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        val w = busyWorld()
        val ids = IntArray(64); val n = w.bodyIds(ids)
        val r = w.events()
        val filter = QueryFilter()
        val hit = RayHit()
        val from = Vec2(f.awayGoalLineX - 6.0, 2.0); val to = Vec2(f.awayGoalLineX + 6.0, 3.0)
        val out = IntArray(8)
        var sink = 0.0
        fun tick() {
            w.step()
            while (r.next()) {
                sink += r.pointX + r.closingSpeed + r.slipExcess + r.tick + r.line + r.type.ordinal + r.bodyA.value
                if (r.wall == ContactView.NO_WALL && (r.type == EventType.Hit || r.type == EventType.ContactBegin)) sink += r.bodyB.value
            }
            if (w.raycast(from, to, filter, hit)) sink += hit.fraction
            if (w.circleCast(from, to, 0.3, filter, hit)) sink += hit.fraction
            sink += w.queryAabb(from.x, 0.0, to.x, 6.0, filter, out)
            sink += w.nearestBodies(from, 3, filter, out)
        }
        // Warm up (class loading, JIT) through every round so every event path has run.
        for (round in 0 until 12) { drive(w, round, ids, n); repeat(120) { tick() } }
        val tid = Thread.currentThread().id
        var allocated = 0L
        for (round in 0 until 6) {
            drive(w, round, ids, n) // builds new intents, so it stays outside the measured window
            val before = mx.getThreadAllocatedBytes(tid)
            repeat(100) { tick() }
            allocated += mx.getThreadAllocatedBytes(tid) - before
        }
        assertEquals(0L, allocated, "bytes allocated by 600 steps with events and queries (sink $sink)")
    }
}
