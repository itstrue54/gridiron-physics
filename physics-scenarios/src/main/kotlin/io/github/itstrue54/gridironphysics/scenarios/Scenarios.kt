package io.github.itstrue54.gridironphysics.scenarios

/** Every scenario, in the order the viewer lists them. Later milestones add the spec's six setups. */
object Scenarios {

    /**
     * M1: 22 players near midfield with random shoves, coasting to a stop under damping (no collisions
     * yet, so they pass through each other), an official walking the near sideline, and a static prop
     * on the home goal line.
     */
    val drift = scenario(
        name = "Drift",
        description = "22 players coast to a stop under damping; official walks the sideline (M1, no collisions)",
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

    val all: List<Scenario> = listOf(drift)
}
