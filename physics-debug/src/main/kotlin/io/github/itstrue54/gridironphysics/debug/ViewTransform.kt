package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.FieldConfig

/** A world-space rectangle (meters) the camera shows. */
data class ViewRect(val minX: Double, val minY: Double, val maxX: Double, val maxY: Double) {
    val width: Double get() = maxX - minX
    val height: Double get() = maxY - minY
    val centerX: Double get() = (minX + maxX) / 2
    val centerY: Double get() = (minY + maxY) / 2

    companion object {
        /** The field plus its runoff. */
        fun wholeWorld(f: FieldConfig) = ViewRect(-f.runoffM, -f.runoffM, f.lengthM + f.runoffM, f.widthM + f.runoffM)
    }
}

/**
 * Maps world meters to canvas pixels. [view] is fitted with one uniform scale and centered. Field +x runs to
 * the right; the near sideline (y = 0) is at the bottom.
 */
class ViewTransform(
    val field: FieldConfig,
    val canvasW: Float,
    val canvasH: Float,
    val marginPx: Float = 16f,
    val view: ViewRect = ViewRect.wholeWorld(field),
) {
    val pxPerMeter: Float = minOf(
        (canvasW - 2 * marginPx) / view.width.toFloat(),
        (canvasH - 2 * marginPx) / view.height.toFloat(),
    ).coerceAtLeast(0.01f)

    private val cx = canvasW / 2
    private val cy = canvasH / 2

    fun toScreenX(x: Double): Float = cx + ((x - view.centerX) * pxPerMeter).toFloat()
    fun toScreenY(y: Double): Float = cy - ((y - view.centerY) * pxPerMeter).toFloat()
    fun toWorldX(sx: Float): Double = (sx - cx).toDouble() / pxPerMeter + view.centerX
    fun toWorldY(sy: Float): Double = (cy - sy).toDouble() / pxPerMeter + view.centerY
    fun len(meters: Double): Float = (meters * pxPerMeter).toFloat()
}
