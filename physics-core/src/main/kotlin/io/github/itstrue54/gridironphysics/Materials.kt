package io.github.itstrue54.gridironphysics

/**
 * 16-bit collision categories. Two bodies collide only when each one's category is in the other's mask.
 * [HOME] is also the default category (any bit works for a game that does not use teams).
 */
object Category {
    const val HOME = 0x0001
    const val AWAY = 0x0002
    const val BALL = 0x0004
    const val OFFICIAL = 0x0008
    const val STATIC = 0x0010
    const val LINE_SENSOR = 0x0020
    const val ALL = 0xFFFF

    /** Player bits. A [BALL] body touching a player is always sensor-only: possession is game logic. */
    const val PLAYERS = HOME or AWAY
}

/** Material ids for [MaterialTable]. Games may use ids up to [MaterialTable.SIZE] − 1 for their own props. */
object Material {
    const val PLAYER = 0
    const val DOWNED_PLAYER = 1

    /** Goal posts, runoff walls, sleds and other props. */
    const val STRUCTURE = 2
    const val BALL = 3

    /** The ground. Used by ball bounces (M6), not by 2D contacts. */
    const val TURF = 4
}

/**
 * Friction and restitution for each material pair. Immutable and symmetric: [with] returns a changed copy.
 * Pairs not listed fall back to μ 0.5, e 0.
 */
class MaterialTable private constructor(private val mu: DoubleArray, private val e: DoubleArray) {

    fun friction(a: Int, b: Int): Double = mu[a * SIZE + b]
    fun restitution(a: Int, b: Int): Double = e[a * SIZE + b]

    fun with(a: Int, b: Int, friction: Double, restitution: Double): MaterialTable {
        require(a in 0 until SIZE && b in 0 until SIZE) { "material ids must be in 0..${SIZE - 1}" }
        require(friction.isFinite() && friction >= 0.0) { "friction must be finite and >= 0: $friction" }
        require(restitution.isFinite() && restitution in 0.0..1.0) { "restitution must be in 0..1: $restitution" }
        val m = mu.copyOf(); val r = e.copyOf()
        m[a * SIZE + b] = friction; m[b * SIZE + a] = friction
        r[a * SIZE + b] = restitution; r[b * SIZE + a] = restitution
        return MaterialTable(m, r)
    }

    companion object {
        const val SIZE = 8

        /** A table from raw row-major [friction] and [restitution] arrays (snapshots and replays), validated like [with]. */
        internal fun fromRaw(friction: DoubleArray, restitution: DoubleArray): MaterialTable {
            require(friction.size == SIZE * SIZE && restitution.size == SIZE * SIZE) { "material table must be $SIZE x $SIZE" }
            for (i in 0 until SIZE * SIZE) {
                require(friction[i].isFinite() && friction[i] >= 0.0) { "friction must be finite and >= 0: ${friction[i]}" }
                require(restitution[i].isFinite() && restitution[i] in 0.0..1.0) { "restitution must be in 0..1: ${restitution[i]}" }
            }
            return MaterialTable(friction, restitution)
        }

        /** The spec's starting values. Tunable, not measured. */
        val DEFAULT: MaterialTable = MaterialTable(DoubleArray(SIZE * SIZE) { 0.5 }, DoubleArray(SIZE * SIZE))
            .with(Material.PLAYER, Material.PLAYER, 0.6, 0.02)
            .with(Material.PLAYER, Material.DOWNED_PLAYER, 0.8, 0.0)
            .with(Material.DOWNED_PLAYER, Material.DOWNED_PLAYER, 0.8, 0.0)
            .with(Material.PLAYER, Material.STRUCTURE, 0.4, 0.1)
            .with(Material.DOWNED_PLAYER, Material.STRUCTURE, 0.4, 0.1)
            .with(Material.BALL, Material.TURF, 0.5, 0.5)
            .with(Material.BALL, Material.STRUCTURE, 0.3, 0.6)
            .with(Material.BALL, Material.PLAYER, 0.3, 0.3)
            .with(Material.BALL, Material.DOWNED_PLAYER, 0.3, 0.3)
    }
}
