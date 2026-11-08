package io.github.itstrue54.gridironphysics

/** The only place yards exist. Everything inside the engine is meters. For the game layer. */
object Yards {
    const val METERS_PER_YARD: Double = 0.9144

    fun toMeters(yards: Double): Double = yards * METERS_PER_YARD
    fun fromMeters(meters: Double): Double = meters / METERS_PER_YARD
}
