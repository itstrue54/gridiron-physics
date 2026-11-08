package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.FieldConfig

/**
 * Maps world meters to canvas pixels. The whole world (field plus runoff) is fitted with one uniform
 * scale and centered. Field +x runs to the right; the near sideline (y = 0) is at the bottom.
 */
class ViewTransform(
    val field: FieldConfig,
    val canvasW: Float,
    val canvasH: Float,
    val marginPx: Float = 16f,
) {
    private val worldW = field.lengthM + 2 * field.runoffM
    private val worldH = field.widthM + 2 * field.runoffM

    val pxPerMeter: Float = minOf(
        (canvasW - 2 * marginPx) / worldW.toFloat(),
        (canvasH - 2 * marginPx) / worldH.toFloat(),
    ).coerceAtLeast(0.01f)

    // Left edge of the runoff and bottom edge of the runoff, in pixels, after centering.
    private val left = (canvasW - worldW.toFloat() * pxPerMeter) / 2
    private val bottom = canvasH - (canvasH - worldH.toFloat() * pxPerMeter) / 2

    fun toScreenX(x: Double): Float = left + ((x + field.runoffM) * pxPerMeter).toFloat()
    fun toScreenY(y: Double): Float = bottom - ((y + field.runoffM) * pxPerMeter).toFloat()
    fun toWorldX(sx: Float): Double = (sx - left).toDouble() / pxPerMeter - field.runoffM
    fun toWorldY(sy: Float): Double = (bottom - sy).toDouble() / pxPerMeter - field.runoffM
    fun len(meters: Double): Float = (meters * pxPerMeter).toFloat()
}
