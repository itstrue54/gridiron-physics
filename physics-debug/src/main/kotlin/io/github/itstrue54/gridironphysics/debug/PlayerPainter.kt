package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.drawscope.translate
import androidx.compose.ui.graphics.lerp
import io.github.itstrue54.gridironphysics.scenarios.Tags

/** Bodies drawn as a person seen from above (players and officials). Everything else draws its raw shape. */
fun isFigure(userData: Int): Boolean = userData == Tags.HOME || userData == Tags.AWAY || userData == Tags.OFFICIAL

private val Glove = Color(0xFFEFD9B4)
private val Pants = Color(0xFFE0E0E0)
private val Mask = Color(0xFFBDBDBD)

/**
 * A top-down football player at canvas point [c], facing [angle] (world radians, +x down field), with collider
 * radius [rPx] in pixels. Upright: shoulder pads, hands and helmet with facemask. [lying] (capsule stance):
 * full length along the heading, head first, with [halfLengthPx] the capsule's half length.
 */
fun DrawScope.drawPlayerFigure(
    c: Offset, angle: Double, rPx: Float, team: Color, official: Boolean, lying: Boolean, halfLengthPx: Float, alpha: Float,
) {
    val jersey = (if (official) Color.White else team).copy(alpha = alpha)
    val helmet = (if (official) Color(0xFF212121) else lerp(team, Color.Black, 0.35f)).copy(alpha = alpha)
    val outline = Color.Black.copy(alpha = 0.55f * alpha)
    val glove = Glove.copy(alpha = alpha)
    // Screen y is flipped, so a world heading of +a is a screen rotation of −a.
    val degrees = (-angle * 180.0 / Math.PI).toFloat()
    translate(c.x, c.y) {
        rotate(degrees, Offset.Zero) {
            if (lying) drawLying(rPx, halfLengthPx, jersey, helmet, outline, glove, official, alpha)
            else drawUpright(rPx, jersey, helmet, outline, glove, official, alpha)
        }
    }
}

private fun DrawScope.drawUpright(r: Float, jersey: Color, helmet: Color, outline: Color, glove: Color, official: Boolean, alpha: Float) {
    // Hands out front, then shoulder pads spanning the full collider width across the heading.
    drawCircle(glove, r * 0.2f, Offset(r * 0.5f, -r * 0.72f))
    drawCircle(glove, r * 0.2f, Offset(r * 0.5f, r * 0.72f))
    val padsTopLeft = Offset(-r * 0.42f, -r)
    val padsSize = Size(r * 0.84f, 2 * r)
    drawOval(jersey, padsTopLeft, padsSize)
    if (official) {
        for (k in -2..2) {
            val y = k * r * 0.38f
            drawLine(Color.Black.copy(alpha = alpha), Offset(-r * 0.38f, y), Offset(r * 0.38f, y), strokeWidth = r * 0.12f)
        }
    }
    drawOval(outline, padsTopLeft, padsSize, style = Stroke(width = (r * 0.06f).coerceAtLeast(1f)))
    // Helmet (or an official's cap), facemask at the front, center stripe.
    val head = Offset(r * 0.06f, 0f)
    drawCircle(helmet, r * 0.44f, head)
    drawCircle(outline, r * 0.44f, head, style = Stroke(width = (r * 0.05f).coerceAtLeast(1f)))
    if (!official) {
        drawLine(Mask.copy(alpha = alpha), Offset(r * 0.5f, -r * 0.28f), Offset(r * 0.5f, r * 0.28f), strokeWidth = r * 0.12f, cap = StrokeCap.Round)
        drawLine(Color.White.copy(alpha = 0.8f * alpha), Offset(-r * 0.36f, 0f), Offset(r * 0.36f, 0f), strokeWidth = r * 0.09f)
    }
}

private fun DrawScope.drawLying(
    r: Float, h: Float, jersey: Color, helmet: Color, outline: Color, glove: Color, official: Boolean, alpha: Float,
) {
    val pants = Pants.copy(alpha = alpha)
    val total = h + r
    // Legs trail behind, arms reach ahead, torso in between, head toward the front cap.
    drawLine(pants, Offset(-h * 0.1f, -r * 0.32f), Offset(-total + r * 0.25f, -r * 0.4f), strokeWidth = r * 0.5f, cap = StrokeCap.Round)
    drawLine(pants, Offset(-h * 0.1f, r * 0.32f), Offset(-total + r * 0.25f, r * 0.4f), strokeWidth = r * 0.5f, cap = StrokeCap.Round)
    drawLine(jersey, Offset(h * 0.5f, -r * 0.62f), Offset(total - r * 0.15f, -r * 0.5f), strokeWidth = r * 0.3f, cap = StrokeCap.Round)
    drawLine(jersey, Offset(h * 0.5f, r * 0.62f), Offset(total - r * 0.15f, r * 0.5f), strokeWidth = r * 0.3f, cap = StrokeCap.Round)
    drawCircle(glove, r * 0.17f, Offset(total - r * 0.1f, -r * 0.5f))
    drawCircle(glove, r * 0.17f, Offset(total - r * 0.1f, r * 0.5f))
    val torsoTopLeft = Offset(-h * 0.25f, -r * 0.85f)
    val torsoSize = Size(h * 0.95f, r * 1.7f)
    drawOval(jersey, torsoTopLeft, torsoSize)
    drawOval(outline, torsoTopLeft, torsoSize, style = Stroke(width = (r * 0.06f).coerceAtLeast(1f)))
    val head = Offset(h * 0.82f, 0f)
    drawCircle(helmet, r * 0.42f, head)
    drawCircle(outline, r * 0.42f, head, style = Stroke(width = (r * 0.05f).coerceAtLeast(1f)))
    if (!official) drawLine(Color.White.copy(alpha = 0.8f * alpha), Offset(head.x - r * 0.34f, 0f), Offset(head.x + r * 0.34f, 0f), strokeWidth = r * 0.09f)
}
