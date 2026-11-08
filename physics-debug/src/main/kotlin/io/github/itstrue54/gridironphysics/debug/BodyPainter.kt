package io.github.itstrue54.gridironphysics.debug

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.unit.sp
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.BodyView
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.scenarios.Tags

private val Home = Color(0xFF1E88E5)
private val Away = Color(0xFFE53935)
private val Prop = Color(0xFF9E9E9E)
private val Velocity = Color(0xFFFFFF8D)
private val Selected = Color(0xFFFFEB3B)
private val Wall = Color(0xFFB0BEC5)
private val ContactPoint = Color(0xFFFF4081)
private val SpeculativePoint = Color(0x88FF80AB)
private val Normal = Color(0xFF00E5FF)

/** Draws every body at its interpolated position, with heading tick and velocity vector. Sleeping bodies are dimmed. */
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
        val base = when {
            b.type == BodyType.Static -> Prop
            b.userData == Tags.OFFICIAL -> Color.White
            b.userData == Tags.AWAY -> Away
            b.userData == Tags.HOME -> Home
            else -> Prop
        }
        val asleep = b.type == BodyType.Dynamic && !b.awake
        val fill = if (asleep) base.copy(alpha = 0.45f) else base
        val shape = b.shape
        val figure = isFigure(b.userData) && (shape is ShapeDef.Circle || shape is ShapeDef.Capsule)
        if (figure) {
            val lying = shape is ShapeDef.Capsule
            val halfLength = if (shape is ShapeDef.Capsule) t.len(shape.halfLength) else 0f
            drawPlayerFigure(
                c, b.angle, t.len(b.radius).coerceAtLeast(3f), if (b.userData == Tags.AWAY) Away else Home,
                official = b.userData == Tags.OFFICIAL, lying = lying, halfLengthPx = halfLength, alpha = if (asleep) 0.55f else 1f,
            )
            if (s.showColliders) drawShape(t, b, x, y, ColliderOutline, outlineOnly = true)
        } else {
            drawShape(t, b, x, y, fill)
        }
        val r = t.len(b.radius).coerceAtLeast(2f)
        // Heading tick (screen y is flipped). Figures show their heading already.
        if (!figure && b.shape !is ShapeDef.Segment) {
            val hx = DMath.cos(b.angle).toFloat()
            val hy = -DMath.sin(b.angle).toFloat()
            drawLine(Color.White, c, Offset(c.x + hx * r, c.y + hy * r), strokeWidth = 2f)
        }
        // Velocity vector: where the body will be in 0.25 s.
        if (b.type != BodyType.Static && (b.vx != 0.0 || b.vy != 0.0)) {
            drawLine(Velocity, c, Offset(c.x + t.len(b.vx * 0.25), c.y - t.len(b.vy * 0.25)), strokeWidth = 1.5f)
        }
        if (s.selected == id) drawCircle(Selected, r + 4f, c, style = Stroke(width = 2.5f))
    }
}

private val ColliderOutline = Color(0xFFFFFF00)

private fun DrawScope.drawShape(t: ViewTransform, b: BodyView, x: Double, y: Double, fill: Color, outlineOnly: Boolean = false) {
    val cos = DMath.cos(b.angle)
    val sin = DMath.sin(b.angle)
    fun sx(lx: Double, ly: Double) = t.toScreenX(x + cos * lx - sin * ly)
    fun sy(lx: Double, ly: Double) = t.toScreenY(y + sin * lx + cos * ly)
    val sensorDash = PathEffect.dashPathEffect(floatArrayOf(6f, 4f))
    when (val shape = b.shape) {
        is ShapeDef.Circle -> {
            val c = Offset(t.toScreenX(x), t.toScreenY(y))
            val r = t.len(shape.radius).coerceAtLeast(2f)
            if (outlineOnly) drawCircle(fill, r, c, style = Stroke(width = 1.5f))
            else if (shape.sensor) drawCircle(fill, r, c, style = Stroke(width = 2f, pathEffect = sensorDash))
            else if (b.type == BodyType.Static) drawCircle(fill, r, c, style = Stroke(width = 2f))
            else { drawCircle(fill, r, c); drawCircle(Color.Black.copy(alpha = 0.5f), r, c, style = Stroke(width = 1f)) }
        }
        is ShapeDef.Capsule -> {
            val h = shape.halfLength
            val p1 = Offset(sx(-h, 0.0), sy(-h, 0.0))
            val p2 = Offset(sx(h, 0.0), sy(h, 0.0))
            if (outlineOnly) {
                // Two end circles plus the two side lines.
                val rr = t.len(shape.radius)
                drawCircle(fill, rr, p1, style = Stroke(width = 1.5f)); drawCircle(fill, rr, p2, style = Stroke(width = 1.5f))
                val ox = (-(sin) * shape.radius); val oy = cos * shape.radius
                drawLine(fill, Offset(sx(-h, 0.0) + t.len(ox), sy(-h, 0.0) - t.len(oy)), Offset(sx(h, 0.0) + t.len(ox), sy(h, 0.0) - t.len(oy)), strokeWidth = 1.5f)
                drawLine(fill, Offset(sx(-h, 0.0) - t.len(ox), sy(-h, 0.0) + t.len(oy)), Offset(sx(h, 0.0) - t.len(ox), sy(h, 0.0) + t.len(oy)), strokeWidth = 1.5f)
            } else {
                drawLine(fill, p1, p2, strokeWidth = 2 * t.len(shape.radius).coerceAtLeast(2f), cap = StrokeCap.Round)
            }
        }
        is ShapeDef.Polygon -> {
            val path = Path()
            for (i in 0 until shape.count) {
                val px = sx(shape.vertex(2 * i), shape.vertex(2 * i + 1))
                val py = sy(shape.vertex(2 * i), shape.vertex(2 * i + 1))
                if (i == 0) path.moveTo(px, py) else path.lineTo(px, py)
            }
            path.close()
            if (shape.sensor) drawPath(path, fill, style = Stroke(width = 2f, pathEffect = sensorDash))
            else { drawPath(path, fill); drawPath(path, Color.Black.copy(alpha = 0.5f), style = Stroke(width = 1f)) }
        }
        is ShapeDef.Segment -> {
            val p1 = Offset(sx(shape.x1, shape.y1), sy(shape.x1, shape.y1))
            val p2 = Offset(sx(shape.x2, shape.y2), sy(shape.x2, shape.y2))
            if (shape.sensor) drawLine(SensorColor, p1, p2, strokeWidth = 3f, pathEffect = sensorDash)
            else drawLine(fill, p1, p2, strokeWidth = 3f)
        }
    }
}

/** The four runoff walls (engine geometry, not bodies). */
fun DrawScope.drawRunoffWalls(t: ViewTransform, enabled: Boolean) {
    if (!enabled) return
    val f = t.field
    val x0 = t.toScreenX(-f.runoffM); val x1 = t.toScreenX(f.lengthM + f.runoffM)
    val y0 = t.toScreenY(-f.runoffM); val y1 = t.toScreenY(f.widthM + f.runoffM)
    drawLine(Wall, Offset(x0, y0), Offset(x1, y0), strokeWidth = 4f)
    drawLine(Wall, Offset(x0, y1), Offset(x1, y1), strokeWidth = 4f)
    drawLine(Wall, Offset(x0, y0), Offset(x0, y1), strokeWidth = 4f)
    drawLine(Wall, Offset(x1, y0), Offset(x1, y1), strokeWidth = 4f)
}

/**
 * Contact points from the last step: solid when touching, faint when only speculative. The cyan normal
 * points from A to B and grows with the normal impulse; impulses above 1 N·s are labeled.
 */
fun DrawScope.drawContacts(t: ViewTransform, s: ViewerState, text: TextMeasurer) {
    val w = s.world
    val style = TextStyle(color = Normal, fontSize = 10.sp)
    for (i in 0 until w.contactCount) {
        val c = w.contact(i)
        for (k in 0 until c.pointCount) {
            val p = Offset(t.toScreenX(c.pointX(k)), t.toScreenY(c.pointY(k)))
            val touching = c.separation(k) < 0.0
            drawCircle(if (touching) ContactPoint else SpeculativePoint, 3f, p)
            if (c.sensor) continue
            val impulse = c.normalImpulse(k)
            val len = 0.3 + (impulse / 400.0).coerceAtMost(1.5)
            drawLine(Normal, p, Offset(p.x + t.len(c.normalX * len), p.y - t.len(c.normalY * len)), strokeWidth = 1.5f)
            if (impulse > 1.0) drawText(text, "%.0f".format(impulse), Offset(p.x + 4f, p.y + 2f), style)
        }
    }
}
