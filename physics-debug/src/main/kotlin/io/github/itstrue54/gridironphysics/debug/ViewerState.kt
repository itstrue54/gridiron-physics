package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.BallState
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.ContactView
import io.github.itstrue54.gridironphysics.EventType
import io.github.itstrue54.gridironphysics.FieldLine
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.scenarios.Scenario
import io.github.itstrue54.gridironphysics.scenarios.Scenarios

/** A point the ball passed through (z is its height), for the trail. */
class TrailPoint(val x: Double, val y: Double, val z: Double)

/** One event drawn on the field for [ViewerState.MARKER_TICKS] ticks. */
class EventMarker(val type: EventType, val tick: Long, val x: Double, val y: Double, val label: String, val pair: Long = 0L)

/** Plain (non-Compose) viewer state, so it can be unit-tested. The UI redraws after every frame. */
class ViewerState(scenario: Scenario = Scenarios.all.first()) {
    var scenario: Scenario = scenario
        set(value) { field = value; reset() }

    var world: World = scenario.build()
        private set

    var paused: Boolean = false

    /** Simulation speed multiplier: 0.25, 1 or 4. */
    var speed: Double = 1.0

    var selected: BodyId? = null

    /** Draw the raw physics shapes over the player figures. A view preference: [reset] keeps it. */
    var showColliders: Boolean = false

    /**
     * Keep the window above other windows. On by default: Windows won't let a window launched from Gradle take
     * focus, so without this it opens and immediately drops behind whatever was in front. [reset] keeps it.
     */
    var alwaysOnTop: Boolean = true

    /** Interpolation fraction between the previous and current tick, for drawing. */
    var alpha: Double = 0.0
        private set

    private val idBuf = IntArray(256)
    private val jointBuf = IntArray(256)

    /** Recent events to draw, oldest first. Contact begin/end and sensor ends are logged but not drawn. */
    val markers: List<EventMarker> get() = markerList
    private val markerList = ArrayList<EventMarker>()

    /** The latest [LOG_SIZE] events as text, newest first. */
    val eventLog: List<String> get() = logList
    private val logList = ArrayList<String>()

    /** Last tick each body slipped, so the log shows where a slip starts instead of one line per tick. */
    private val lastSlip = HashMap<Int, Long>()

    /** Where the ball has been since its latest launch (in the air, bouncing and rolling), oldest first. */
    val ballTrail: List<TrailPoint> get() = trail
    private val trail = ArrayList<TrailPoint>()
    private var trailTick = -1L
    private var lastBallState = BallState.Dead

    fun reset() {
        world = scenario.build()
        markerList.clear()
        logList.clear()
        lastSlip.clear()
        trail.clear()
        trailTick = -1L
        lastBallState = BallState.Dead
        selected = null
        alpha = 0.0
        follow = true
    }

    // ---------------------------------------------------------------- camera

    /** When on, the camera frames every moving body each frame. Wheel zoom and drag turn it off. */
    var follow: Boolean = true

    private var manualView: ViewRect = ViewRect.wholeWorld(world.field)

    /** The world rectangle to draw this frame. */
    fun viewRect(): ViewRect = if (follow) actionRect() else manualView

    /** Zooms by [factor] (> 1 zooms in) about the world point ([wx], [wy]), which stays under the cursor. */
    fun zoomBy(factor: Double, wx: Double, wy: Double) {
        val r = viewRect()
        manualView = ViewRect(
            wx - (wx - r.minX) / factor, wy - (wy - r.minY) / factor,
            wx + (r.maxX - wx) / factor, wy + (r.maxY - wy) / factor,
        )
        follow = false
    }

    /** Moves the camera by ([dx], [dy]) meters. */
    fun panBy(dx: Double, dy: Double) {
        val r = viewRect()
        manualView = ViewRect(r.minX + dx, r.minY + dy, r.maxX + dx, r.maxY + dy)
        follow = false
    }

    fun showFullField() {
        manualView = ViewRect.wholeWorld(world.field)
        follow = false
    }

    /** Bounding box of every non-static body, padded, and never smaller than [MIN_VIEW_M] on a side. */
    private fun actionRect(): ViewRect {
        var x0 = Double.MAX_VALUE; var y0 = Double.MAX_VALUE
        var x1 = -Double.MAX_VALUE; var y1 = -Double.MAX_VALUE
        val n = count
        for (i in 0 until n) {
            val b = world.body(BodyId(idBuf[i]))
            if (b.type == BodyType.Static) continue
            val r = b.radius
            x0 = minOf(x0, b.x - r); y0 = minOf(y0, b.y - r)
            x1 = maxOf(x1, b.x + r); y1 = maxOf(y1, b.y + r)
        }
        val ball = world.ball()
        if (ball.state != BallState.Dead) {
            x0 = minOf(x0, ball.x - BALL_PAD_M); y0 = minOf(y0, ball.y - BALL_PAD_M)
            x1 = maxOf(x1, ball.x + BALL_PAD_M); y1 = maxOf(y1, ball.y + BALL_PAD_M)
        }
        if (x0 > x1) return ViewRect.wholeWorld(world.field)
        val cx = (x0 + x1) / 2; val cy = (y0 + y1) / 2
        val hw = maxOf((x1 - x0) / 2 + PAD_M, MIN_VIEW_M / 2)
        val hh = maxOf((y1 - y0) / 2 + PAD_M, MIN_VIEW_M / 2)
        return ViewRect(cx - hw, cy - hh, cx + hw, cy + hh)
    }

    companion object {
        /** Smallest side of the follow window (m). */
        const val MIN_VIEW_M = 6.0

        /** Margin around the moving bodies (m). */
        const val PAD_M = 1.5

        /** How long an event marker stays on the field: 1.5 s. */
        const val MARKER_TICKS = 90L

        /** Lines in the event log. */
        const val LOG_SIZE = 14

        /** Extra room around the ball in the follow window (m), so its lifted drawing and label stay on screen. */
        const val BALL_PAD_M = 3.0

        /** Points kept in the ball trail (8 s). */
        const val TRAIL_SIZE = 480

        private val LINE_NAMES = arrayOf("near sideline", "far sideline", "home end line", "away end line", "home goal line", "away goal line")

        fun lineName(line: Int): String = if (line in 0 until FieldLine.COUNT) LINE_NAMES[line] else "line " + line

    }

    /** Advances exactly one tick (used while paused). Drawn at the new position, not interpolated. */
    fun stepOnce() {
        world.step()
        alpha = 1.0
        collectEvents()
    }

    /** Called once per rendered frame with the real elapsed time. */
    fun frame(frameSeconds: Double) {
        if (paused) return
        alpha = world.advance(frameSeconds * speed)
        collectEvents()
    }

    /** Adds the ball's position after each new tick to the trail; a new flight from a hand or a dead ball starts a new trail. */
    private fun recordBall() {
        val b = world.ball()
        val st = b.state
        if (st == BallState.Airborne && (lastBallState == BallState.Dead || lastBallState == BallState.Carried)) trail.clear()
        if (world.tick == 0L && trail.isEmpty() && st == BallState.Airborne) trail.add(TrailPoint(b.prevX, b.prevY, b.prevZ))
        lastBallState = st
        if (world.tick == trailTick) return
        trailTick = world.tick
        if (st == BallState.Airborne || st == BallState.Loose) {
            trail.add(TrailPoint(b.x, b.y, b.z))
            if (trail.size > TRAIL_SIZE) trail.removeAt(0)
        }
    }

    /** Drains the world's events into markers and the log, and drops markers older than [MARKER_TICKS]. */
    private fun collectEvents() {
        recordBall()
        val r = world.events()
        while (r.next()) {
            val type = r.type
            var x = r.pointX
            var y = r.pointY
            var mark = true
            val text = when (type) {
                EventType.Hit -> "HIT %.1f m/s %.0f N·s".format(r.closingSpeed, r.normalImpulse)
                EventType.Trip -> "TRIP %.1f m/s".format(r.speed)
                EventType.Slip -> "slip +%.1f m/s²".format(r.slipExcess)
                EventType.GoalLineCrossed -> "GOAL LINE " + if (r.direction > 0) "→" else "←"
                EventType.OutOfBounds -> "OUT (" + lineName(r.line) + ")"
                EventType.SensorBegin -> lineName(r.line)
                EventType.SensorEnd -> { mark = false; "left " + lineName(r.line) }
                EventType.GrappleBroken -> "GRAPPLE BROKEN %.0f N".format(r.force)
                EventType.BlockShed -> "BLOCK SHED %.0f N".format(r.force)
                EventType.Down -> if (r.ballCarrier) "DOWN, spot x = %.2f m".format(r.forwardProgressX) else "down"
                EventType.BallBounce -> "bounce %.1f m/s".format(r.speed)
                EventType.KickResult -> "KICK " + r.kickResult.name + " (z %.1f m)".format(r.pointZ)
                EventType.CatchOpportunity -> "CATCH CHANCE %.1f m/s, z %.1f m".format(r.speed, r.pointZ)
                EventType.ContactBegin, EventType.ContactEnd -> {
                    mark = false
                    val other = if (r.wall != ContactView.NO_WALL) "wall " + r.wall else "#" + r.bodyB.value
                    (if (type == EventType.ContactBegin) "touch " else "untouch ") + other + if (r.sensor) " (sensor)" else ""
                }
            }
            val slipGoesOn = type == EventType.Slip && lastSlip.put(r.bodyA.value, r.tick) == r.tick - 1
            if (!slipGoesOn) logList.add(0, "" + r.tick + "  #" + r.bodyA.value + "  " + text)
            if (logList.size > LOG_SIZE) logList.removeAt(logList.size - 1)
            if (!mark) continue
            // Ring the player for a catch chance, not the ball; mark line events where the body is now.
            if (type == EventType.CatchOpportunity || type == EventType.Slip || type == EventType.OutOfBounds || type == EventType.SensorBegin || type == EventType.Down) {
                if (r.bodyA == world.ballId && world.ball().state != BallState.Loose) {
                    x = world.ball().x; y = world.ball().y // the ball left the ground again since
                } else {
                    val b = world.body(r.bodyA)
                    x = b.x; y = b.y
                }
            }
            if (type == EventType.GrappleBroken || type == EventType.BlockShed) {
                // Where the joint was: midway between its bodies.
                val ax = world.body(r.bodyA).x; val ay = world.body(r.bodyA).y
                val b = world.body(r.bodyB)
                x = (ax + b.x) / 2; y = (ay + b.y) / 2
            }
            if (type == EventType.Hit) {
                // A pair that bounces and re-engages hits again on the same spot: draw the ring, keep the first label.
                val pair = (r.bodyA.value.toLong() shl 32) or r.bodyB.value.toLong()
                val repeat = markerList.any { it.type == EventType.Hit && it.pair == pair }
                markerList.add(EventMarker(type, r.tick, x, y, if (repeat) "" else text, pair))
                continue
            }
            markerList.add(EventMarker(type, r.tick, x, y, text))
        }
        val now = world.tick
        markerList.removeAll { now - it.tick >= MARKER_TICKS }
    }

    /** Live joint ids, ascending. The returned array is reused; read only the first [jointCount] entries. */
    fun jointIds(): IntArray = jointBuf
    val jointCount: Int get() = world.jointIds(jointBuf)

    /** Live body ids, ascending. The returned array is reused; read only the first [count] entries. */
    fun ids(): IntArray = idBuf
    val count: Int get() = world.bodyIds(idBuf)

    /** The body whose circle contains the world point, nearest center first, or null. Never the ball. */
    fun pick(wx: Double, wy: Double): BodyId? {
        var best: BodyId? = null
        var bestD2 = Double.MAX_VALUE
        val n = count
        for (i in 0 until n) {
            // Not the loose ball: it stops being a body when it leaves the ground (the panel shows it separately).
            if (idBuf[i] == world.ballId.value) continue
            val b = world.body(BodyId(idBuf[i]))
            val dx = b.x - wx
            val dy = b.y - wy
            val d2 = dx * dx + dy * dy
            val r = b.radius
            if (d2 <= r * r && d2 < bestD2) {
                bestD2 = d2
                best = BodyId(idBuf[i])
            }
        }
        return best
    }
}
