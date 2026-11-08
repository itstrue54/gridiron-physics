package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.JointId
import io.github.itstrue54.gridironphysics.JointType

private val Rigid = Color(0xFFB0BEC5)
private val ProgressColor = Color(0xFFFFD740)
private val progressDash = PathEffect.dashPathEffect(floatArrayOf(14f, 8f))

/** Green at no load, through yellow, to red at the break force (or shed threshold). */
fun loadColor(load: Double): Color {
    val l = load.coerceIn(0.0, 1.0).toFloat()
    return if (l < 0.5f) {
        val k = l / 0.5f
        Color(0.18f + 0.82f * k, 0.80f + 0.12f * k, 0.25f - 0.15f * k)
    } else {
        val k = (l - 0.5f) / 0.5f
        Color(1f, 0.92f - 0.82f * k, 0.10f)
    }
}

/**
 * Joints between interpolated body centers. Grapples and blocks are colored by load against their break force or
 * shed threshold and labeled with the percentage; grapples are drawn as rope (thinner), blocks as a thick bar.
 * Distance and weld joints have no break force and are drawn gray.
 */
fun DrawScope.drawJoints(t: ViewTransform, s: ViewerState, text: TextMeasurer) {
    val w = s.world
    val ids = s.jointIds()
    val n = s.jointCount
    val a = s.alpha
    for (i in 0 until n) {
        val j = w.joint(JointId(ids[i]))
        val type = j.type
        val load = j.load
        val ida = j.bodyA; val idb = j.bodyB
        val (ax, ay) = lerp(s, ida, a)
        val (bx, by) = lerp(s, idb, a)
        val p1 = Offset(t.toScreenX(ax), t.toScreenY(ay))
        val p2 = Offset(t.toScreenX(bx), t.toScreenY(by))
        when (type) {
            JointType.Grapple, JointType.Block -> {
                val color = loadColor(load)
                val width = if (type == JointType.Block) t.len(0.16).coerceIn(4f, 14f) else t.len(0.07).coerceIn(2.5f, 7f)
                drawLine(Color.Black.copy(alpha = 0.55f), p1, p2, strokeWidth = width + 2f, cap = StrokeCap.Round)
                drawLine(color, p1, p2, strokeWidth = width, cap = StrokeCap.Round)
                // A quarter of the way from A, so the labels of joints sharing a body (a double team) stay apart.
                val mid = Offset(p1.x + (p2.x - p1.x) * 0.25f, p1.y + (p2.y - p1.y) * 0.25f)
                val label = (if (type == JointType.Grapple) "grip " else "shed ") + "%.0f%%".format(100 * load)
                drawText(text, label, Offset(mid.x + 6f, mid.y - 18f), TextStyle(color = color, fontSize = 10.sp, fontWeight = FontWeight.Bold))
            }
            else -> drawLine(Rigid, p1, p2, strokeWidth = 2f, cap = StrokeCap.Round)
        }
    }
}

private fun lerp(s: ViewerState, id: BodyId, a: Double): Pair<Double, Double> {
    val b = s.world.body(id)
    return Pair(b.prevX + (b.x - b.prevX) * a, b.prevY + (b.y - b.prevY) * a)
}

/** A dashed gold line across the field at the ball carrier's forward progress. */
fun DrawScope.drawForwardProgress(t: ViewTransform, s: ViewerState, text: TextMeasurer) {
    val x = s.world.forwardProgressX
    if (x.isNaN()) return
    val f = t.field
    val sx = t.toScreenX(x)
    drawLine(ProgressColor.copy(alpha = 0.8f), Offset(sx, t.toScreenY(0.0)), Offset(sx, t.toScreenY(f.widthM)), strokeWidth = 2f, pathEffect = progressDash)
    // Labeled at the bottom of the canvas, clear of the pile and its labels.
    drawText(text, "forward progress", Offset(sx + 4f, size.height - 20f), TextStyle(color = ProgressColor, fontSize = 10.sp, fontWeight = FontWeight.Bold))
}
