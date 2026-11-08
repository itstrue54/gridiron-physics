package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.BodyDef
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.Category
import io.github.itstrue54.gridironphysics.FieldConfig
import io.github.itstrue54.gridironphysics.Material
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.WorldConfig
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.SplitMix64

/** `userData` convention used by scenarios and the viewer's colors. Games use their own handles. */
object Tags {
    const val HOME = 0
    const val AWAY = 1
    const val OFFICIAL = 2
    const val PROP = 3
}

/** A named, seeded world setup that runs headless in tests and visually in the viewer. */
class Scenario(
    val name: String,
    val description: String,
    val seed: Long,
    private val field: FieldConfig,
    private val config: WorldConfig,
    private val setup: ScenarioBuilder.() -> Unit,
) {
    /** Builds a fresh world. Same scenario, same world, every time. */
    fun build(): World {
        val world = World(config, field, seed)
        ScenarioBuilder(world, SplitMix64(seed)).setup()
        return world
    }

    override fun toString(): String = name
}

fun scenario(
    name: String,
    description: String,
    seed: Long = 1L,
    field: FieldConfig = FieldConfig.PRO,
    config: WorldConfig = WorldConfig(),
    setup: ScenarioBuilder.() -> Unit,
): Scenario = Scenario(name, description, seed, field, config, setup)

class ScenarioBuilder(val world: World, val rng: SplitMix64) {
    val field: FieldConfig get() = world.field

    fun player(
        x: Double,
        y: Double,
        vx: Double = 0.0,
        vy: Double = 0.0,
        mass: Double = 100.0,
        radius: Double = 0.34,
        damping: Double = 0.5,
        tag: Int = Tags.HOME,
    ): BodyId = world.createBody(
        BodyDef(
            x = x, y = y, vx = vx, vy = vy, mass = mass, linearDamping = damping, angularDamping = damping,
            angle = if (tag == Tags.AWAY) DMath.PI else 0.0,
            shape = ShapeDef.Circle(radius), userData = tag,
            categoryBits = if (tag == Tags.AWAY) Category.AWAY else Category.HOME,
        ),
    )

    /** Any body, for setups the helpers above don't cover. */
    fun body(def: BodyDef): BodyId = world.createBody(def)

    fun official(x: Double, y: Double, vx: Double, vy: Double): BodyId = world.createBody(
        BodyDef(
            type = BodyType.Kinematic, x = x, y = y, vx = vx, vy = vy, shape = ShapeDef.Circle(0.30),
            userData = Tags.OFFICIAL, categoryBits = Category.OFFICIAL,
        ),
    )

    fun prop(x: Double, y: Double, radius: Double): BodyId = world.createBody(
        BodyDef(
            type = BodyType.Static, x = x, y = y, shape = ShapeDef.Circle(radius), userData = Tags.PROP,
            categoryBits = Category.STATIC, material = Material.STRUCTURE,
        ),
    )
}
