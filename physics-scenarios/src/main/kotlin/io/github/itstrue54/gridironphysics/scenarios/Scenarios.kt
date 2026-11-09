package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.AthleteParams
import io.github.itstrue54.gridironphysics.BlockDef
import io.github.itstrue54.gridironphysics.BodyDef
import io.github.itstrue54.gridironphysics.GrappleDef
import io.github.itstrue54.gridironphysics.LocomotionIntent
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.Category
import io.github.itstrue54.gridironphysics.FlightMode
import io.github.itstrue54.gridironphysics.Material
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.Stance
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.Vec3

/** Every scenario, in the order the viewer lists them (newest milestone first). */
object Scenarios {

    private const val YARD = 0.9144

    /**
     * M6: a go route down the near sideline in a crosswind blowing toward that sideline (and a little into the
     * passer's face). The receiver is already at full speed with a corner trailing him; the quarterback throws a 13 m
     * high spiral led to where the receiver will be, solved against the drag and the wind, so the ball starts out over
     * the field and drifts back onto him. Both players have catch windows: the viewer marks each CatchOpportunity.
     * Nobody decides the catch here, so the ball comes down past the receiver and bounces.
     */
    val deepPass = scenario(
        name = "Deep pass to the sideline with wind",
        description = "A led 13 m spiral to a go route on the near sideline, aimed upwind against a 5 m/s crosswind (M6)",
        seed = 29L,
    ) {
        val wr = AthleteParams(maxSpeed = 9.2, maxAccel = 8.0, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val cb = AthleteParams(maxSpeed = 9.0, maxAccel = 8.0, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val los = field.homeGoalLineX + 25 * YARD
        world.setWind(-1.5, -5.0)
        val qb = athlete(los - 7 * YARD, field.nearHashY, 100.0, 0.34, Tags.HOME, wr, LocomotionIntent.velocity(0.0, 0.0))
        val receiver = athlete(los + 6.0, 4.0, 88.0, 0.32, Tags.HOME, wr, LocomotionIntent.velocity(9.0, 0.0), vx = 9.0)
        val corner = athlete(los + 4.5, 5.5, 86.0, 0.32, Tags.AWAY, cb, LocomotionIntent.velocity(8.8, -0.2), vx = 8.8)
        world.setReach(receiver, 1.0, 2.6)
        world.setReach(corner, 1.0, 2.6)
        // Lead him: guess the flight time, aim at where he will be then, re-time the solved throw, repeat.
        val q = world.body(qb)
        val from = Vec3(q.x + 0.3, q.y, 1.9)
        val r = world.body(receiver)
        val rx = r.x; val ry = r.y
        val target = Vec3()
        val v = Vec3()
        var t = 2.5
        repeat(8) {
            target.set(rx + 9.0 * t, ry, 1.5)
            world.launchVelocityFor(from, target, 13.0, FlightMode.Spiral, v)
            t = world.timeToArrive(from, v, FlightMode.Spiral, target)
        }
        world.launchBall(from, v, FlightMode.Spiral)
    }

    /**
     * M6: a punt from midfield aimed at the away 5-yard line, end over end with 4.5 s of hang time. It lands inside
     * the 10, bounces with the seeded ±35° yaw of a prolate ball and rolls on as the loose ball toward the goal line
     * while two gunners close in and the returner stays back.
     */
    val punt = scenario(
        name = "Punt that bounces near the goal line",
        description = "A 50-yard punt with 4.5 s hang time lands inside the 10 and bounces toward the goal line (M6)",
        seed = 31L,
    ) {
        val gunner = AthleteParams(maxSpeed = 9.3, maxAccel = 8.0, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val cy = field.widthM / 2
        val target = Vec3(field.awayGoalLineX - 5 * YARD, cy + 5.0, 0.0)
        val punter = player(field.midfieldX - 14 * YARD, cy, mass = 95.0, radius = 0.33, damping = 2.0)
        val p = world.body(punter)
        // Gunners already running at 7 m/s straight for a spot short of the landing, one from each sideline.
        fun gunnerAt(y: Double, ty: Double) {
            val x = field.midfieldX + 2.0
            val gx = target.x - 3.0
            val d = DMath.hypot2(gx - x, ty - y)
            athlete(x, y, 90.0, 0.32, Tags.HOME, gunner, LocomotionIntent.point(gx, ty), vx = 7.0 * (gx - x) / d, vy = 7.0 * (ty - y) / d)
        }
        gunnerAt(3.5, target.y - 3.0)
        gunnerAt(field.widthM - 3.5, target.y + 3.0)
        player(field.awayGoalLineX - 12 * YARD, cy - 8.0, mass = 92.0, radius = 0.32, damping = 2.0, tag = Tags.AWAY)
        val from = Vec3(p.x + 0.5, p.y, 1.0)
        val v = world.launchVelocityFor(from, target, 25.0, FlightMode.EndOverEnd, Vec3())
        world.launchBall(from, v, FlightMode.EndOverEnd)
    }

    /**
     * M6: a 45-yard field goal (snap 7 yards behind a line at the 28) that comes off the left upright: the kick is
     * solved at 27 m/s end over end straight at the upright 7 m up, so it clangs off it (KickResult HitPost) and falls
     * back toward the end zone. Holder, kicker and a few linemen stand for scale.
     */
    val fieldGoal = scenario(
        name = "45-yard field goal that hits the upright",
        description = "A 27 m/s end-over-end kick from 45 yards clangs off the left upright: KickResult HitPost (M6)",
        seed = 37L,
    ) {
        val cy = field.widthM / 2
        val spotX = field.lengthM - 45 * YARD
        val spotY = cy - 1.5
        player(spotX - 0.6, spotY - 0.9, mass = 90.0, radius = 0.32, damping = 3.0) // holder
        player(spotX - 2.4, spotY - 1.6, mass = 88.0, radius = 0.32, damping = 3.0) // kicker
        for (k in -3..3) player(spotX + 6.4, spotY + 0.9 * k, mass = 140.0, radius = 0.37, damping = 3.0)
        for (k in -2..2) player(spotX + 7.4, spotY + 1.1 * k, mass = 135.0, radius = 0.37, damping = 3.0, tag = Tags.AWAY)
        val from = Vec3(spotX, spotY, 0.0)
        // The left upright (kicker's left, +y going toward +x), a few centimetres inside its axis.
        val target = Vec3(field.lengthM, cy + field.uprightSpacingM / 2 - 0.04, 7.0)
        val v = Vec3()
        check(world.launchVelocityFor(from, target, 27.0, false, FlightMode.EndOverEnd, v)) { "45 yards is in range at 27 m/s" }
        world.launchBall(from, v, FlightMode.EndOverEnd)
    }

    /**
     * M5: two wrap-ups at once. A 95 kg carrier at 8 m/s keeps driving downfield with two tacklers wrapped on him
     * (one from behind on each side, carried along almost at his speed); their drag stacks and brings him to a stop within a few yards, and the
     * forward-progress line marks his best spot. Below them a slower carrier pulls away from a weak arm tackle,
     * which strains (the rope turns red) and breaks (GrappleBroken).
     */
    val tackleWithGrapple = scenario(
        name = "Tackle with grapple",
        description = "Two tacklers' drag stacks and stops a driving carrier; a weak arm tackle strains and breaks (M5)",
        seed = 19L,
    ) {
        val runner = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val mid = field.midfieldX
        val cy = field.widthM / 2
        world.setAttackDirection(1)
        val carrier = athlete(mid - 8.0, cy, 95.0, 0.32, Tags.HOME, runner, LocomotionIntent.velocity(8.5, 0.0), vx = 8.0)
        world.setBallCarrier(carrier)
        val t1 = player(mid - 8.45, cy + 0.5, vx = 7.5, mass = 110.0, radius = 0.34, damping = 1.5, tag = Tags.AWAY)
        val t2 = player(mid - 8.3, cy - 0.62, vx = 7.2, mass = 105.0, radius = 0.34, damping = 1.5, tag = Tags.AWAY)
        world.createGrapple(t1, carrier, GrappleDef(breakForce = 3000.0, dragForce = 700.0))
        world.createGrapple(t2, carrier, GrappleDef(breakForce = 3000.0, dragForce = 700.0))
        // The arm tackle: a tackler with his heels dug in (heavy damping) behind a carrier who turns on the burners.
        // The faster they go, the harder the tackler's feet drag, so the pull ramps up until the grip goes.
        val c2 = athlete(mid - 8.0, cy - 7.0, 90.0, 0.32, Tags.HOME, runner, LocomotionIntent.velocity(9.0, 0.0), vx = 0.5)
        val arm = player(mid - 8.68, cy - 7.0, vx = 0.5, mass = 100.0, radius = 0.34, damping = 2.5, tag = Tags.AWAY)
        world.createGrapple(arm, c2, GrappleDef(breakForce = 470.0))
    }

    /**
     * M5: the spec's double-team block on a 150 kg lineman. Two blockers angled in at ±30° push 1500 N each; the
     * lineman pushes back with 2400 N split across the two blocks, so leverage and numbers drive him back until
     * the push fades with speed. Below them a one-on-one defender rips sideways and sheds his block (BlockShed).
     */
    val doubleTeam = scenario(
        name = "Double-team block on a 150 kg lineman",
        description = "Two angled blockers drive a 150 kg lineman back; a one-on-one defender rips sideways and sheds (M5)",
        seed = 23L,
    ) {
        val mid = field.midfieldX
        val cy = field.widthM / 2
        val gap = 0.72 // engaged distance between centers
        val a = DMath.PI / 6
        val ca = DMath.cos(a); val sa = DMath.sin(a)
        fun lineman(x: Double, y: Double, angle: Double, mass: Double, tag: Int) = body(
            BodyDef(
                x = x, y = y, angle = angle, mass = mass, linearDamping = 0.3, angularDamping = SPIN_DAMPING,
                shape = ShapeDef.Circle(0.36), userData = tag, categoryBits = if (tag == Tags.AWAY) Category.AWAY else Category.HOME,
            ),
        )
        val nose = lineman(mid + 0.5, cy, DMath.PI, 150.0, Tags.AWAY)
        val left = lineman(mid + 0.5 - gap * ca, cy - gap * sa, a, 140.0, Tags.HOME)
        val right = lineman(mid + 0.5 - gap * ca, cy + gap * sa, -a, 135.0, Tags.HOME)
        for (b in listOf(left, right)) {
            world.createBlock(b, nose, BlockDef(blockerPushForce = 1500.0, defenderPushForce = 1200.0, shedThreshold = 900.0, driveSpeed = 2.0))
        }
        // One-on-one: the defender rips toward the near sideline and sheds.
        val blocker = lineman(mid + 0.5 - gap, cy - 6.0, 0.0, 130.0, Tags.HOME)
        val rusher = lineman(mid + 0.5, cy - 6.0, DMath.PI, 125.0, Tags.AWAY) // facing his blocker
        world.setAthlete(rusher, AthleteParams(maxSpeed = 7.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 8.0, turnRateAtMax = 2.5))
        world.setIntent(rusher, LocomotionIntent.point(mid + 0.5, cy - 10.0))
        world.createBlock(blocker, rusher, BlockDef(blockerPushForce = 1300.0, defenderPushForce = 1300.0, shedThreshold = 400.0))
    }

    /**
     * M4: one of each event near the away end zone. Two players collide (Hit), a runner trips over a Down player
     * (Trip), a receiver breaks the goal line and runs out over the end line (GoalLineCrossed, OutOfBounds), and a
     * runner cuts hard toward the near sideline (Slip, then OutOfBounds). The viewer overlays each event.
     */
    val eventLab = scenario(
        name = "Event lab",
        description = "Hit, trip, slip, goal line and out of bounds near the away end zone; events overlaid (M4)",
        seed = 17L,
    ) {
        val fast = AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val slow = AthleteParams(maxSpeed = 7.0, maxAccel = 6.5, maxDecel = 8.0, turnRateAtRest = 10.0, turnRateAtMax = 2.5)
        val gl = field.awayGoalLineX
        // Hit: head-on, slightly off-center.
        athlete(gl - 11.0, 30.0, 92.0, 0.32, Tags.HOME, fast, LocomotionIntent.velocity(8.5, 0.0), vx = 8.5)
        athlete(gl - 3.0, 30.15, 112.0, 0.36, Tags.AWAY, slow, LocomotionIntent.velocity(-7.0, 0.0), vx = -7.0)
        // Trip: a runner straight into a player lying on the ground.
        val down = player(gl - 5.5, 20.0, mass = 105.0, radius = 0.34, damping = 2.0, tag = Tags.AWAY)
        world.setStance(down, Stance.Down)
        athlete(gl - 11.5, 20.2, 95.0, 0.33, Tags.HOME, fast, LocomotionIntent.velocity(8.0, 0.0), vx = 8.0)
        // Goal line, then out over the end line.
        athlete(gl - 5.5, 40.0, 88.0, 0.32, Tags.HOME, fast, LocomotionIntent.velocity(8.0, 3.0), vx = 8.0, vy = 3.0)
        // Slip: a hard cut from running downfield to running at the near sideline.
        athlete(gl - 12.5, 9.0, 95.0, 0.33, Tags.HOME, fast, LocomotionIntent.velocity(0.0, -8.5), vx = 8.5)
    }

    /**
     * M3: the spec's head-on tackle on locomotion: a 90 kg back at 9 m/s and a 115 kg linebacker at 7 m/s run
     * at each other (both already at top speed), slightly off-center so the hit has a glancing component. Their
     * intents keep driving after the hit: the back downfield, the linebacker upfield.
     */
    val headOnTackle = scenario(
        name = "Head-on tackle",
        description = "90 kg back at 9 m/s vs 115 kg linebacker at 7 m/s; both keep driving through the hit (M3)",
        seed = 11L,
    ) {
        val mid = field.midfieldX
        val cy = field.widthM / 2
        val back = AthleteParams(maxSpeed = 9.0, maxAccel = 7.5, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val linebacker = AthleteParams(maxSpeed = 7.0, maxAccel = 6.5, maxDecel = 8.0, turnRateAtRest = 10.0, turnRateAtMax = 2.5)
        athlete(mid - 6.0, cy, 90.0, 0.32, Tags.HOME, back, LocomotionIntent.velocity(back.maxSpeed, 0.0), vx = back.maxSpeed)
        athlete(mid + 6.0, cy + 0.15, 115.0, 0.36, Tags.AWAY, linebacker, LocomotionIntent.velocity(-linebacker.maxSpeed, 0.0), vx = -linebacker.maxSpeed)
    }

    /**
     * M3: a carrier at top speed down the near sideline and a slower pursuer taking an angle from infield. The
     * pursuer aims at the intercept point (where a straight line at his top speed meets the carrier's path), so
     * he cuts the carrier off at the sideline instead of chasing him from behind.
     */
    val sidelineRun = scenario(
        name = "Sideline run",
        description = "Carrier at 9.3 m/s down the sideline; a 8.8 m/s pursuer takes the angle and cuts him off (M3)",
        seed = 13L,
    ) {
        val carrier = AthleteParams(maxSpeed = 9.3, maxAccel = 8.0, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val pursuer = AthleteParams(maxSpeed = 8.8, maxAccel = 7.5, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val cx = field.midfieldX - 15.0
        val cy = 3.0
        val px = cx + 22.0
        val py = 15.0
        // Intercept: |d + vc·t| = vp·t with d = carrier − pursuer, vc = (vCarrier, 0). Take the earliest t > 0.
        val vc = carrier.maxSpeed
        val vp = pursuer.maxSpeed
        val dx = cx - px
        val dy = cy - py
        val a = vc * vc - vp * vp
        val b = 2 * dx * vc
        val c = dx * dx + dy * dy
        val t = (-b - DMath.sqrt(b * b - 4 * a * c)) / (2 * a)
        val ax = dx + vc * t
        val len = DMath.hypot2(ax, dy)
        val ux = ax / len
        val uy = dy / len
        athlete(cx, cy, 90.0, 0.32, Tags.HOME, carrier, LocomotionIntent.velocity(vc, 0.0), vx = vc)
        athlete(px, py, 100.0, 0.34, Tags.AWAY, pursuer, LocomotionIntent.velocity(vp * ux, vp * uy), vx = vp * ux, vy = vp * uy)
    }

    /**
     * M2: one of every shape and wall interaction. A runner bounces off the near runoff wall, a player drives a
     * sled, a diver (capsule) slides into a downed player, a player glances off a goal-post base (static
     * polygon), two players collide at an angle, and a runner crosses a line sensor (static segment) without
     * being stopped. Everything has high damping so the lab settles and falls asleep.
     */
    val contactLab = scenario(
        name = "Contact lab",
        description = "Walls, sled, capsules, post base, line sensor; everything settles and sleeps (M2)",
        seed = 5L,
    ) {
        val d = 1.0 // linear damping: everyone coasts to rest within a few seconds
        val spin = SPIN_DAMPING
        fun runner(x: Double, y: Double, vx: Double, vy: Double, tag: Int = Tags.HOME) = body(
            BodyDef(
                x = x, y = y, vx = vx, vy = vy, mass = 95.0, linearDamping = d, angularDamping = spin,
                angle = DMath.atan2(vy, vx), shape = ShapeDef.Circle(0.33), userData = tag,
                categoryBits = if (tag == Tags.AWAY) Category.AWAY else Category.HOME,
            ),
        )
        // Into the near runoff wall.
        runner(30.0, -3.0, 0.0, -8.0)
        // Driving a sled.
        runner(40.0, 20.0, 7.0, 0.0)
        body(
            BodyDef(
                x = 43.0, y = 20.0, mass = 200.0, linearDamping = 2.0, angularDamping = spin,
                shape = ShapeDef.box(0.5, 1.0), userData = Tags.PROP, categoryBits = Category.STATIC,
                material = Material.STRUCTURE,
            ),
        )
        // A diver slides into a downed player.
        body(
            BodyDef(
                x = 50.0, y = 22.0, vx = 6.0, mass = 100.0, linearDamping = d, angularDamping = spin,
                shape = ShapeDef.Capsule(0.6, 0.3), userData = Tags.HOME, categoryBits = Category.HOME,
            ),
        )
        body(
            BodyDef(
                x = 53.0, y = 22.3, angle = DMath.PI / 2, mass = 110.0, linearDamping = d, angularDamping = spin,
                shape = ShapeDef.Capsule(0.6, 0.32), userData = Tags.AWAY, categoryBits = Category.AWAY,
                material = Material.DOWNED_PLAYER,
            ),
        )
        // A goal-post base (static polygon) and a player glancing off it.
        body(
            BodyDef(
                type = BodyType.Static, x = 60.0, y = 27.0,
                shape = ShapeDef.Polygon(doubleArrayOf(-0.6, -0.4, 0.6, -0.4, 0.8, 0.0, 0.6, 0.4, -0.6, 0.4, -0.8, 0.0)),
                userData = Tags.PROP, categoryBits = Category.STATIC, material = Material.STRUCTURE,
            ),
        )
        runner(57.5, 25.8, 5.0, 2.4)
        // Two players colliding at an angle.
        runner(35.0, 32.0, 6.0, 2.0, Tags.HOME)
        runner(40.0, 34.5, -4.0, -3.0, Tags.AWAY) // paths cross at (38, 33)
        // A runner crossing a line sensor: reported, never stopped.
        body(
            BodyDef(
                type = BodyType.Static, x = 70.0, y = 30.0, shape = ShapeDef.Segment(0.0, -5.0, 0.0, 5.0, sensor = true),
                userData = Tags.PROP, categoryBits = Category.LINE_SENSOR,
            ),
        )
        runner(67.5, 30.0, 5.0, 0.0)
    }

    /**
     * M1: 22 players near midfield with random shoves, coasting to a stop under damping, colliding since M2,
     * an official walking the near sideline, and a static prop on the home goal line.
     */
    val drift = scenario(
        name = "Drift",
        description = "22 players shoved apart coast to a stop, colliding; official walks the sideline",
        seed = 7L,
    ) {
        val mid = field.midfieldX
        val near = field.nearHashY
        val far = field.farHashY
        // Offense (HOME) lined up just short of midfield, defense (AWAY) mirrored past it.
        val offense = listOf(
            -1.0 to 0.0, -1.0 to 1.2, -1.0 to -1.2, -1.0 to 2.4, -1.0 to -2.4, // line
            -1.0 to 8.0, -1.0 to -12.0, // ends / receivers
            -5.0 to 0.0, -7.0 to 0.0, // QB, back
            -2.0 to 14.0, -2.0 to -14.0, // wideouts
        )
        val defense = listOf(
            1.5 to 0.6, 1.5 to -0.6, 1.5 to 2.0, 1.5 to -2.0, // line
            5.0 to 0.0, 5.0 to 4.0, 5.0 to -4.0, // backers
            3.0 to 14.0, 3.0 to -14.0, // corners
            12.0 to 6.0, 12.0 to -6.0, // safeties
        )
        val centerY = (near + far) / 2
        fun shove() = rng.nextDouble(-8.0, 8.0) / 1.4142135623730951
        offense.forEach { (dx, dy) ->
            player(mid + dx, centerY + dy, shove(), shove(), rng.nextDouble(85.0, 150.0), rng.nextDouble(0.30, 0.38), tag = Tags.HOME)
        }
        defense.forEach { (dx, dy) ->
            player(mid + dx, centerY + dy, shove(), shove(), rng.nextDouble(85.0, 150.0), rng.nextDouble(0.30, 0.38), tag = Tags.AWAY)
        }
        official(x = mid - 15.0, y = -2.0, vx = 1.2, vy = 0.0)
        prop(x = field.homeGoalLineX, y = field.widthM / 2, radius = 0.5)
    }

    /**
     * M8: the benchmark world, 25 dynamic players on locomotion. Five offensive linemen block five rushers who drive at
     * the quarterback; four receivers run routes with defensive backs on them, two backs swing out against three
     * linebackers and a safety, and the quarterback throws deep at the snap. The pass lands, bounces and rolls loose
     * through traffic. physics-bench steps this world; the viewer shows it at 4x.
     */
    val scrimmage = scenario(
        name = "25-player scrimmage (benchmark)",
        description = "Benchmark world: 25 players, five blocks, routes, a deep pass that bounces loose through traffic (M8)",
        seed = 31L,
    ) {
        val los = field.midfieldX
        val cy = field.widthM / 2
        val line = AthleteParams(maxSpeed = 6.8, maxAccel = 5.0, maxDecel = 7.0, turnRateAtRest = 8.0, turnRateAtMax = 2.5)
        val skill = AthleteParams(maxSpeed = 9.0, maxAccel = 8.0, maxDecel = 8.5, turnRateAtRest = 12.0, turnRateAtMax = 2.5)
        val back = AthleteParams(maxSpeed = 8.2, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 10.0, turnRateAtMax = 2.5)
        val qb = athlete(los - 6.0, cy, 100.0, 0.34, Tags.HOME, skill, LocomotionIntent.velocity(-1.0, 0.0, 0.4))
        for (k in 0 until 5) {
            val y = cy + (k - 2) * 1.5
            val o = athlete(los - 0.75, y, 140.0 + 3 * k, 0.37, Tags.HOME, line, LocomotionIntent.point(los - 2.0, y, 0.6))
            val d = athlete(los + 0.75, y + 0.2, 130.0 + 4 * k, 0.36, Tags.AWAY, line, LocomotionIntent.point(los - 6.0, cy, 1.0))
            world.createBlock(o, d, BlockDef(blockerPushForce = 1200.0, defenderPushForce = 1100.0 + 60 * k, shedThreshold = 1800.0 + 400 * k, driveSpeed = 2.0))
        }
        val wideY = doubleArrayOf(3.0, 9.0, field.widthM - 9.0, field.widthM - 3.0)
        for (k in 0 until 4) {
            val vy = if (k < 2) 0.8 else -0.8
            athlete(los - 0.5, wideY[k], 88.0, 0.32, Tags.HOME, skill, LocomotionIntent.velocity(8.8, vy * (k % 2)))
            athlete(los + 5.0, wideY[k] + 0.4, 86.0, 0.32, Tags.AWAY, skill, LocomotionIntent.velocity(8.6, vy * (k % 2)))
        }
        athlete(los - 5.0, cy - 1.5, 98.0, 0.34, Tags.HOME, back, LocomotionIntent.velocity(3.0, -6.0))
        athlete(los - 5.0, cy + 1.5, 100.0, 0.34, Tags.HOME, back, LocomotionIntent.velocity(3.0, 6.0))
        for (k in 0 until 3) athlete(los + 5.0, cy + (k - 1) * 5.0, 108.0, 0.35, Tags.AWAY, back, LocomotionIntent.point(los - 3.0, cy + (k - 1) * 9.0))
        athlete(los + 16.0, cy, 92.0, 0.33, Tags.AWAY, skill, LocomotionIntent.point(los + 30.0, wideY[1]))
        // The deep ball, thrown at the snap toward the second receiver's route.
        val q = world.body(qb)
        val from = Vec3(q.x + 0.3, q.y, 1.9)
        val v = Vec3()
        world.launchVelocityFor(from, Vec3(los + 30.0, wideY[1] + 2.0, 1.2), 11.0, FlightMode.Spiral, v)
        world.launchBall(from, v, FlightMode.Spiral)
    }

    val all: List<Scenario> = listOf(scrimmage, deepPass, punt, fieldGoal, tackleWithGrapple, doubleTeam, headOnTackle, sidelineRun, eventLab, contactLab, drift)

    /** Angular damping for coasting bodies (no locomotion holds their heading). */
    private const val SPIN_DAMPING = 8.0
}
