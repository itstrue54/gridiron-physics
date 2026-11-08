package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.AthleteParams
import io.github.itstrue54.gridironphysics.BodyDef
import io.github.itstrue54.gridironphysics.LocomotionIntent
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.Category
import io.github.itstrue54.gridironphysics.Material
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.Stance
import io.github.itstrue54.gridironphysics.math.DMath

/** Every scenario, in the order the viewer lists them. Later milestones add the spec's other setups. */
object Scenarios {

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

    val all: List<Scenario> = listOf(headOnTackle, sidelineRun, eventLab, contactLab, drift)

    /** Angular damping for coasting bodies (no locomotion holds their heading). */
    private const val SPIN_DAMPING = 8.0
}
