package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.BodyDef
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.Category
import io.github.itstrue54.gridironphysics.Material
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.math.DMath

/** Every scenario, in the order the viewer lists them. Later milestones add the spec's other setups. */
object Scenarios {

    /**
     * M2: the spec's head-on tackle, collision only (locomotion arrives in M3): a 90 kg back at 9 m/s meets a
     * 115 kg linebacker at 7 m/s, slightly off-center so the hit has a glancing component.
     */
    val headOnTackle = scenario(
        name = "Head-on tackle",
        description = "90 kg back at 9 m/s vs 115 kg linebacker at 7 m/s, collision only (M2)",
        seed = 11L,
    ) {
        val mid = field.midfieldX
        val cy = field.widthM / 2
        player(x = mid - 5.0, y = cy, vx = 9.0, mass = 90.0, radius = 0.32, tag = Tags.HOME)
        player(x = mid + 5.0, y = cy + 0.15, vx = -7.0, mass = 115.0, radius = 0.36, tag = Tags.AWAY)
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
        val d = 1.5 // damping
        fun runner(x: Double, y: Double, vx: Double, vy: Double, tag: Int = Tags.HOME) = body(
            BodyDef(
                x = x, y = y, vx = vx, vy = vy, mass = 95.0, linearDamping = d, angularDamping = d,
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
                x = 43.0, y = 20.0, mass = 200.0, linearDamping = 2.0, angularDamping = 2.0,
                shape = ShapeDef.box(0.5, 1.0), userData = Tags.PROP, categoryBits = Category.STATIC,
                material = Material.STRUCTURE,
            ),
        )
        // A diver slides into a downed player.
        body(
            BodyDef(
                x = 50.0, y = 22.0, vx = 6.0, mass = 100.0, linearDamping = d, angularDamping = d,
                shape = ShapeDef.Capsule(0.6, 0.3), userData = Tags.HOME, categoryBits = Category.HOME,
            ),
        )
        body(
            BodyDef(
                x = 53.0, y = 22.3, angle = DMath.PI / 2, mass = 110.0, linearDamping = d, angularDamping = d,
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
        runner(56.0, 25.0, 5.0, 2.4)
        // Two players colliding at an angle.
        runner(35.0, 32.0, 6.0, 2.0, Tags.HOME)
        runner(40.0, 36.0, -4.0, -3.0, Tags.AWAY)
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

    val all: List<Scenario> = listOf(headOnTackle, contactLab, drift)
}
