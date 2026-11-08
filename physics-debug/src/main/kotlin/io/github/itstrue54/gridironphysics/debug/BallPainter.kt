package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import io.github.itstrue54.gridironphysics.BallState

private val Leather = Color(0xFF9C5A2E)
private val Laces = Color(0xFFFFF3E0)
private val TrailColor = Color(0xFFFFCC80)
private val LabelColor = Color(0xFFFFE0B2)
private val tether = PathEffect.dashPathEffect(floatArrayOf(4f, 4f))

/**
 * How far up the screen the ball is drawn above its shadow for height [z] (m): a fixed oblique view in which a meter
 * of height reads as half a meter of ground. The shadow stays at the ball's true ground position.
 */
fun ballLift(z: Double, pxPerMeter: Float): Float = (z * LIFT * pxPerMeter).toFloat()

private const val LIFT = 0.5

/**
 * The ball: its shadow on the ground at (x, y), the ball itself lifted by its height (prolate, pointed along its
 * travel) with a dashed tether down to the shadow and a height label, and a faint trail of where it has been since
 * the launch. A loose ball sits on its shadow; a carried ball rides at its carrier.
 */
fun DrawScope.drawBall(t: ViewTransform, s: ViewerState, text: TextMeasurer) {
    val w = s.world
    val b = w.ball()
    if (b.state == BallState.Dead && s.ballTrail.isEmpty()) return
    val a = s.alpha
    val x = b.prevX + (b.x - b.prevX) * a
    val y = b.prevY + (b.y - b.prevY) * a
    val z = b.prevZ + (b.z - b.prevZ) * a
    val ppm = t.pxPerMeter

    // Trail: the lifted path (where the ball was drawn).
    val trail = s.ballTrail
    if (trail.size > 1) {
        val path = Path()
        trail.forEachIndexed { i, p ->
            val px = t.toScreenX(p.x)
            val py = t.toScreenY(p.y) - ballLift(p.z, ppm)
            if (i == 0) path.moveTo(px, py) else path.lineTo(px, py)
        }
        drawPath(path, TrailColor.copy(alpha = 0.45f), style = Stroke(width = 1.5f))
    }

    val gx = t.toScreenX(x)
    val gy = t.toScreenY(y)
    // Shadow: wider and fainter the higher the ball.
    val sr = t.len(0.16 + 0.025 * z).coerceAtLeast(4f)
    val shade = (0.55f - 0.012f * z.toFloat()).coerceIn(0.15f, 0.55f)
    drawOval(Color.Black.copy(alpha = shade), Offset(gx - sr, gy - sr * 0.6f), Size(2 * sr, 1.2f * sr))
    val bx = gx
    val by = gy - ballLift(z, ppm)
    if (z > 0.3) drawLine(LabelColor.copy(alpha = 0.6f), Offset(gx, gy), Offset(bx, by), strokeWidth = 1f, pathEffect = tether)

    // The ball, pointed along its ground velocity (screen y is flipped).
    val len = (t.len(0.28) * (1f + z.toFloat() / 40f)).coerceAtLeast(9f)
    val wid = len * 0.6f
    val heading = if (b.vx != 0.0 || b.vy != 0.0) Math.toDegrees(Math.atan2(-b.vy, b.vx)).toFloat() else 0f
    rotate(heading, Offset(bx, by)) {
        drawOval(Leather, Offset(bx - len / 2, by - wid / 2), Size(len, wid))
        drawOval(Color.Black.copy(alpha = 0.6f), Offset(bx - len / 2, by - wid / 2), Size(len, wid), style = Stroke(width = 1f))
        drawLine(Laces, Offset(bx - len * 0.18f, by), Offset(bx + len * 0.18f, by), strokeWidth = 1.5f)
    }

    // Height label in the air; the state otherwise (nothing while carried: the carrier shows where it is).
    val label = when (b.state) {
        BallState.Airborne -> "%.1f m".format(z)
        BallState.Loose -> "loose"
        BallState.Carried -> ""
        BallState.Dead -> "dead"
    }
    if (label.isNotEmpty()) {
        drawText(text, label, Offset(bx + len / 2 + 4f, by - 16f), TextStyle(color = LabelColor, fontSize = 11.sp, fontWeight = FontWeight.Bold))
    }
}
