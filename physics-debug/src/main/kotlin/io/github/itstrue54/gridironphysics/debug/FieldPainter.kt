package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.unit.sp
import io.github.itstrue54.gridironphysics.Yards

private val Runoff = Color(0xFF1E4D2B)
private val Grass = Color(0xFF2E7D32)
private val EndZone = Color(0xFF256B2A)
private val Chalk = Color(0xCCFFFFFF)
private val ChalkFaint = Color(0x66FFFFFF)

/** Sensor lines (sidelines, end lines, goal lines) are drawn dashed amber: bodies cross them; they are not walls. */
val SensorColor = Color(0xFFFFB300)

fun DrawScope.drawField(t: ViewTransform, text: TextMeasurer) {
    val f = t.field
    // Runoff and field.
    drawRect(Runoff, Offset(t.toScreenX(-f.runoffM), t.toScreenY(f.widthM + f.runoffM)), Size(t.len(f.lengthM + 2 * f.runoffM), t.len(f.widthM + 2 * f.runoffM)))
    drawRect(Grass, Offset(t.toScreenX(0.0), t.toScreenY(f.widthM)), Size(t.len(f.lengthM), t.len(f.widthM)))
    drawRect(EndZone, Offset(t.toScreenX(0.0), t.toScreenY(f.widthM)), Size(t.len(f.endZoneDepthM), t.len(f.widthM)))
    drawRect(EndZone, Offset(t.toScreenX(f.awayGoalLineX), t.toScreenY(f.widthM)), Size(t.len(f.endZoneDepthM), t.len(f.widthM)))

    val top = t.toScreenY(f.widthM)
    val bottom = t.toScreenY(0.0)
    // Yard lines every 5 yards between the goal lines, thicker every 10.
    for (yd in 5 until 100 step 5) {
        val sx = t.toScreenX(f.homeGoalLineX + Yards.toMeters(yd.toDouble()))
        drawLine(if (yd % 10 == 0) Chalk else ChalkFaint, Offset(sx, top), Offset(sx, bottom), strokeWidth = if (yd % 10 == 0) 2f else 1f)
    }
    // Hash marks every yard at both hashes, plus sideline ticks.
    val hashLen = t.len(0.6)
    for (yd in 1 until 100) {
        if (yd % 5 == 0) continue
        val sx = t.toScreenX(f.homeGoalLineX + Yards.toMeters(yd.toDouble()))
        for (hy in doubleArrayOf(f.nearHashY, f.farHashY)) {
            val sy = t.toScreenY(hy)
            drawLine(Chalk, Offset(sx, sy - hashLen / 2), Offset(sx, sy + hashLen / 2), strokeWidth = 1f)
        }
        drawLine(ChalkFaint, Offset(sx, bottom), Offset(sx, bottom - hashLen), strokeWidth = 1f)
        drawLine(ChalkFaint, Offset(sx, top), Offset(sx, top + hashLen), strokeWidth = 1f)
    }
    // Yard numbers every 10 (10..50..10).
    val style = TextStyle(color = Chalk, fontSize = (t.pxPerMeter * 1.6f).coerceIn(8f, 28f).sp)
    for (yd in 10 until 100 step 10) {
        val label = (if (yd <= 50) yd else 100 - yd).toString()
        val sx = t.toScreenX(f.homeGoalLineX + Yards.toMeters(yd.toDouble()))
        val layout = text.measure(label, style)
        for (ny in doubleArrayOf(Yards.toMeters(12.0), f.widthM - Yards.toMeters(12.0))) {
            drawText(layout, topLeft = Offset(sx - layout.size.width / 2f, t.toScreenY(ny) - layout.size.height / 2f))
        }
    }

    // Sensor lines.
    val dash = PathEffect.dashPathEffect(floatArrayOf(10f, 6f))
    fun sensor(x0: Double, y0: Double, x1: Double, y1: Double) =
        drawLine(SensorColor, Offset(t.toScreenX(x0), t.toScreenY(y0)), Offset(t.toScreenX(x1), t.toScreenY(y1)), strokeWidth = 3f, pathEffect = dash)
    sensor(0.0, 0.0, f.lengthM, 0.0) // near sideline
    sensor(0.0, f.widthM, f.lengthM, f.widthM) // far sideline
    sensor(0.0, 0.0, 0.0, f.widthM) // home end line
    sensor(f.lengthM, 0.0, f.lengthM, f.widthM) // away end line
    sensor(f.homeGoalLineX, 0.0, f.homeGoalLineX, f.widthM)
    sensor(f.awayGoalLineX, 0.0, f.awayGoalLineX, f.widthM)

    // Goal posts (top-down: crossbar + uprights over each end line).
    val half = f.uprightSpacingM / 2
    val mid = f.widthM / 2
    for (x in doubleArrayOf(0.0, f.lengthM)) {
        // Gooseneck: an arm from the crossbar back to the pole on its padded base behind the end line.
        val back = if (x == 0.0) -f.goalPostOffsetM else f.goalPostOffsetM
        val bx = t.toScreenX(x + back)
        val by = t.toScreenY(mid)
        val bh = t.len(f.goalPostBaseHalfM)
        drawRect(Color(0xFF455A64), Offset(bx - bh, by - bh), Size(2 * bh, 2 * bh))
        drawLine(Color.Yellow, Offset(t.toScreenX(x), by), Offset(bx, by), strokeWidth = 3f)
        drawCircle(Color.Yellow, t.len(0.12).coerceAtLeast(2f), Offset(bx, by))
        drawLine(Color.Yellow, Offset(t.toScreenX(x), t.toScreenY(mid - half)), Offset(t.toScreenX(x), t.toScreenY(mid + half)), strokeWidth = 4f)
        drawCircle(Color.Yellow, t.len(0.15).coerceAtLeast(2f), Offset(t.toScreenX(x), t.toScreenY(mid - half)))
        drawCircle(Color.Yellow, t.len(0.15).coerceAtLeast(2f), Offset(t.toScreenX(x), t.toScreenY(mid + half)))
    }
}
