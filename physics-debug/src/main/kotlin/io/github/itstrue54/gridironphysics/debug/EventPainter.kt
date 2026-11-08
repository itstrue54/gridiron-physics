package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import io.github.itstrue54.gridironphysics.EventType

private val HitColor = Color(0xFFFF9100)
private val TripColor = Color(0xFFE040FB)
private val SlipColor = Color(0xFFFFEA00)
private val OutColor = Color(0xFFFF1744)
private val GoalColor = Color(0xFFFFD740)
private val SensorColorMark = Color(0xFF80D8FF)

/**
 * Event markers from [ViewerState.markers], fading out over [ViewerState.MARKER_TICKS]: an expanding orange ring
 * for a hit, a magenta cross for a trip, yellow dots along a slipping player's path, a red box when a player goes
 * out, a gold diamond where a goal line was crossed, and a small cyan ring when a body touches a field line.
 */
fun DrawScope.drawEvents(t: ViewTransform, s: ViewerState, text: TextMeasurer) {
    val now = s.world.tick
    for (m in s.markers) {
        val age = ((now - m.tick).toFloat() / ViewerState.MARKER_TICKS).coerceIn(0f, 1f)
        val fade = 1f - age
        val c = Offset(t.toScreenX(m.x), t.toScreenY(m.y))
        when (m.type) {
            EventType.Hit -> {
                val r = t.len(0.4 + 0.9 * age).coerceAtLeast(6f)
                drawCircle(HitColor.copy(alpha = fade), r, c, style = Stroke(width = 3f))
                label(text, m.label, c, r, HitColor, fade)
            }
            EventType.Trip -> {
                val h = t.len(0.45).coerceAtLeast(6f)
                val col = TripColor.copy(alpha = fade)
                drawLine(col, Offset(c.x - h, c.y - h), Offset(c.x + h, c.y + h), strokeWidth = 3f)
                drawLine(col, Offset(c.x - h, c.y + h), Offset(c.x + h, c.y - h), strokeWidth = 3f)
                label(text, m.label, c, h, TripColor, fade, below = true) // a trip is a hit too: keep the labels apart
            }
            EventType.Slip -> drawCircle(SlipColor.copy(alpha = 0.8f * fade), t.len(0.12).coerceAtLeast(2f), c)
            EventType.OutOfBounds -> {
                val h = t.len(0.5).coerceAtLeast(7f)
                drawRect(OutColor.copy(alpha = fade), Offset(c.x - h, c.y - h), Size(2 * h, 2 * h), style = Stroke(width = 3f))
                label(text, m.label, c, h, OutColor, fade)
            }
            EventType.GoalLineCrossed -> {
                val h = t.len(0.45).coerceAtLeast(6f)
                val col = GoalColor.copy(alpha = fade)
                drawLine(col, Offset(c.x, c.y - h), Offset(c.x + h, c.y), strokeWidth = 3f)
                drawLine(col, Offset(c.x + h, c.y), Offset(c.x, c.y + h), strokeWidth = 3f)
                drawLine(col, Offset(c.x, c.y + h), Offset(c.x - h, c.y), strokeWidth = 3f)
                drawLine(col, Offset(c.x - h, c.y), Offset(c.x, c.y - h), strokeWidth = 3f)
                label(text, m.label, c, h, GoalColor, fade)
            }
            EventType.SensorBegin -> drawCircle(SensorColorMark.copy(alpha = 0.7f * fade), t.len(0.25).coerceAtLeast(4f), c, style = Stroke(width = 1.5f))
            else -> {}
        }
    }
}

private fun DrawScope.label(text: TextMeasurer, s: String, c: Offset, r: Float, color: Color, fade: Float, below: Boolean = false) {
    if (s.isEmpty()) return
    val style = TextStyle(color = color.copy(alpha = fade), fontSize = 11.sp, fontWeight = FontWeight.Bold)
    drawText(text, s, Offset(c.x + r + 3f, if (below) c.y + r + 2f else c.y - r - 14f), style)
}
