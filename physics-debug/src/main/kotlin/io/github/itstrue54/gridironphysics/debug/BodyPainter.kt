package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.scenarios.Tags

private val Home = Color(0xFF1E88E5)
private val Away = Color(0xFFE53935)
private val Prop = Color(0xFF9E9E9E)
private val Velocity = Color(0xFFFFFF8D)
private val Selected = Color(0xFFFFEB3B)

/** Draws every body at its interpolated position, with heading tick and velocity vector. */
fun DrawScope.drawBodies(t: ViewTransform, s: ViewerState) {
    val w = s.world
    val ids = s.ids()
    val n = s.count
    val a = s.alpha
    for (i in 0 until n) {
        val id = BodyId(ids[i])
        val b = w.body(id)
        val x = b.prevX + (b.x - b.prevX) * a
        val y = b.prevY + (b.y - b.prevY) * a
        val c = Offset(t.toScreenX(x), t.toScreenY(y))
        val r = t.len(b.radius).coerceAtLeast(2f)

        when {
            b.type == BodyType.Static -> drawCircle(Prop, r, c, style = Stroke(width = 2f))
            b.userData == Tags.OFFICIAL -> {
                drawCircle(Color.White, r, c)
                drawLine(Color.Black, Offset(c.x - r * 0.7f, c.y), Offset(c.x + r * 0.7f, c.y), strokeWidth = r * 0.35f)
            }
            else -> {
                drawCircle(if (b.userData == Tags.AWAY) Away else if (b.userData == Tags.HOME) Home else Prop, r, c)
                drawCircle(Color.Black.copy(alpha = 0.5f), r, c, style = Stroke(width = 1f))
            }
        }
        // Heading tick (screen y is flipped).
        val hx = DMath.cos(b.angle).toFloat()
        val hy = -DMath.sin(b.angle).toFloat()
        drawLine(Color.White, c, Offset(c.x + hx * r, c.y + hy * r), strokeWidth = 2f)
        // Velocity vector: where the body will be in 0.25 s.
        if (b.type != BodyType.Static && (b.vx != 0.0 || b.vy != 0.0)) {
            drawLine(Velocity, c, Offset(c.x + t.len(b.vx * 0.25), c.y - t.len(b.vy * 0.25)), strokeWidth = 1.5f)
        }
        if (s.selected == id) drawCircle(Selected, r + 4f, c, style = Stroke(width = 2.5f))
    }
}
