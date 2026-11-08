package io.github.itstrue54.gridironphysics

/**
 * Every field constant, in meters. Origin is the back corner of the home end zone on the near sideline;
 * +x runs down the field, +y across it. Pro and college layouts are presets, not code changes.
 */
data class FieldConfig(
    val lengthM: Double = 109.728,
    val widthM: Double = 48.768,
    val endZoneDepthM: Double = 9.144,
    val hashSpacingM: Double = 5.639,
    val crossbarHeightM: Double = 3.048,
    val uprightSpacingM: Double = 5.639,
    val uprightHeightAboveCrossbarM: Double = 10.668,
    val runoffM: Double = 6.0,
) {
    init {
        require(lengthM > 2 * endZoneDepthM && widthM > 0.0 && endZoneDepthM > 0.0) { "bad field size" }
        require(hashSpacingM in 0.0..widthM && uprightSpacingM > 0.0 && runoffM >= 0.0) { "bad field markings" }
    }

    val homeGoalLineX: Double get() = endZoneDepthM
    val awayGoalLineX: Double get() = lengthM - endZoneDepthM
    val midfieldX: Double get() = lengthM / 2
    val nearHashY: Double get() = (widthM - hashSpacingM) / 2
    val farHashY: Double get() = (widthM + hashSpacingM) / 2

    companion object {
        val PRO = FieldConfig()
        val COLLEGE = FieldConfig(hashSpacingM = 12.192)
    }
}
