package io.github.itstrue54.gridironphysics.debug

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.withFrameNanos
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.input.pointer.PointerEventType
import androidx.compose.ui.input.pointer.onPointerEvent
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.WindowPlacement
import androidx.compose.ui.window.WindowPosition
import androidx.compose.ui.window.application
import androidx.compose.ui.window.rememberWindowState
import io.github.itstrue54.gridironphysics.BallState
import io.github.itstrue54.gridironphysics.scenarios.Scenario
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import java.awt.FileDialog
import java.awt.Frame
import java.io.File

/** Optional argument: a `.gpr` replay to open (`./gradlew :physics-debug:run -Preplay=path/to/play.gpr`). */
fun main(args: Array<String>) = application {
    var onTop by remember { mutableStateOf(true) }
    val replayPath = args.firstOrNull { it.isNotBlank() }
    Window(
        onCloseRequest = ::exitApplication,
        title = "gridiron-physics viewer",
        state = rememberWindowState(placement = WindowPlacement.Maximized, width = 1500.dp, height = 860.dp, position = WindowPosition(Alignment.Center)),
        alwaysOnTop = onTop,
    ) {
        MaterialTheme(colorScheme = darkColorScheme()) {
            Surface(Modifier.fillMaxSize()) { ViewerApp(onTopChanged = { onTop = it }, replayFile = replayPath?.let { File(it) }) }
        }
    }
}

@OptIn(ExperimentalComposeUiApi::class)
@Composable
fun ViewerApp(
    initial: Scenario = Scenarios.all.first(),
    showColliders: Boolean = false,
    onTopChanged: (Boolean) -> Unit = {},
    replayFile: File? = null,
    replayEntry: Int = -1,
) {
    val state = remember {
        ViewerState(initial).also { st ->
            st.showColliders = showColliders
            if (replayFile != null) st.loadReplay(ReplayFiles.read(replayFile), replayFile.name)
            if (replayEntry >= 0) ReplayFiles.builtIn()[replayEntry].let { e -> st.loadReplay(e.load(), e.label) }
        }
    }
    // Bumped every frame and on every control action; reading it makes the canvas and panel redraw.
    var frameNo by remember { mutableLongStateOf(0L) }
    val textMeasurer = rememberTextMeasurer()
    var transform by remember { mutableStateOf<ViewTransform?>(null) }

    LaunchedEffect(Unit) {
        var last = 0L
        while (true) {
            withFrameNanos { now ->
                if (last != 0L) state.frame(((now - last) / 1e9).coerceAtMost(0.25))
                last = now
                frameNo++
            }
        }
    }

    Column(Modifier.fillMaxSize()) {
        TopBar(state, frameNo, onTopChanged) { frameNo++ }
        Row(Modifier.fillMaxSize()) {
            Canvas(
                Modifier.weight(1f).fillMaxHeight().clipToBounds().background(Color(0xFF102915))
                    .pointerInput(Unit) {
                        detectTapGestures { p ->
                            val t = transform ?: return@detectTapGestures
                            state.selected = state.pick(t.toWorldX(p.x), t.toWorldY(p.y))
                            frameNo++
                        }
                    }
                    .pointerInput(Unit) {
                        // Drag to pan: the world moves with the pointer.
                        detectDragGestures { _, d ->
                            val t = transform ?: return@detectDragGestures
                            state.panBy(-d.x / t.pxPerMeter.toDouble(), d.y / t.pxPerMeter.toDouble())
                            frameNo++
                        }
                    }
                    .onPointerEvent(PointerEventType.Scroll) { e ->
                        // Wheel up zooms in about the cursor.
                        val t = transform ?: return@onPointerEvent
                        val ch = e.changes.first()
                        val factor = if (ch.scrollDelta.y < 0f) 1.25 else 1.0 / 1.25
                        state.zoomBy(factor, t.toWorldX(ch.position.x), t.toWorldY(ch.position.y))
                        frameNo++
                    },
            ) {
                @Suppress("UNUSED_EXPRESSION") frameNo
                val t = ViewTransform(state.world.field, size.width, size.height, view = state.viewRect())
                transform = t
                drawField(t, textMeasurer)
                drawRunoffWalls(t, state.world.config.runoffWalls)
                drawForwardProgress(t, state, textMeasurer)
                drawJoints(t, state, textMeasurer)
                drawBodies(t, state)
                drawBall(t, state, textMeasurer)
                drawContacts(t, state, textMeasurer)
                drawEvents(t, state, textMeasurer)
            }
            InspectPanel(state, frameNo)
        }
    }
}

@Composable
private fun TopBar(state: ViewerState, frameNo: Long, onTopChanged: (Boolean) -> Unit, changed: () -> Unit) {
    var menuOpen by remember { mutableStateOf(false) }
    // Read the frame counter so the bar recomposes every frame (tick, Play/Pause, replay status); Compose skips a
    // composable whose parameters it never reads.
    @Suppress("UNUSED_EXPRESSION") frameNo
    // Two fixed-height lines: controls (scroll sideways if the window is narrow), then status. Nothing may wrap,
    // or the bar grows taller than the window and pushes the field off screen.
    Column(Modifier.fillMaxWidth().padding(horizontal = 8.dp, vertical = 4.dp)) {
    Row(
        Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Button(onClick = { state.paused = !state.paused; changed() }) { Text(if (state.paused) "Play" else "Pause") }
        OutlinedButton(onClick = { state.stepOnce(); changed() }, enabled = state.paused) { Text("Step") }
        listOf(0.25, 1.0, 4.0).forEach { sp ->
            FilterChip(
                selected = state.speed == sp,
                onClick = { state.speed = sp; changed() },
                label = { Text(if (sp == 0.25) "0.25×" else "${sp.toInt()}×") },
            )
        }
        Box {
            OutlinedButton(onClick = { menuOpen = true }) { Text(if (state.replay == null) "Scenario: ${state.scenario.name}" else "Scenario…") }
            DropdownMenu(expanded = menuOpen, onDismissRequest = { menuOpen = false }) {
                Scenarios.all.forEach { sc ->
                    DropdownMenuItem(text = { Text(sc.name) }, onClick = { state.scenario = sc; menuOpen = false; changed() })
                }
            }
        }
        ReplayMenu(state, changed)
        OutlinedButton(onClick = { state.reset(); changed() }) { Text("Reset") }
        FilterChip(selected = state.follow, onClick = { state.follow = !state.follow; changed() }, label = { Text("Follow") })
        OutlinedButton(onClick = { state.showFullField(); changed() }) { Text("Full field") }
        FilterChip(selected = state.showColliders, onClick = { state.showColliders = !state.showColliders; changed() }, label = { Text("Colliders") })
        FilterChip(
            selected = state.alwaysOnTop,
            onClick = { state.alwaysOnTop = !state.alwaysOnTop; onTopChanged(state.alwaysOnTop); changed() },
            label = { Text("On top") },
        )
    }
    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            "tick ${state.world.tick}   t = ${"%.2f".format(state.world.tick / 60.0)} s",
            fontFamily = FontFamily.Monospace, fontSize = 13.sp, maxLines = 1, softWrap = false,
        )
        Spacer(Modifier.width(16.dp))
        val replay = state.replay
        if (replay == null) {
            Text(
                state.scenario.description + "   (wheel: zoom, drag: pan)",
                Modifier.weight(1f), fontSize = 12.sp, maxLines = 1, softWrap = false, overflow = TextOverflow.Ellipsis,
            )
        } else {
            val p = state.player!!
            val div = state.divergentTick
            val text = when {
                div != null -> "DIVERGED at tick $div: ${p.divergence}"
                p.finished -> "replay verified: all ${replay.tickCount} recorded hashes match"
                else -> "replay OK so far: ${p.ticksPlayed} of ${replay.tickCount} hashes match"
            }
            Text(
                "${state.replayName}  ticks ${replay.startTick}–${replay.endTick}   $text",
                Modifier.weight(1f), fontSize = 12.sp, maxLines = 1, softWrap = false, overflow = TextOverflow.Ellipsis,
                color = if (div != null) Color(0xFFFF6E6E) else Color(0xFF8BE28B),
                fontFamily = FontFamily.Monospace,
            )
        }
    }
    }
}

@Composable
private fun ReplayMenu(state: ViewerState, changed: () -> Unit) {
    var open by remember { mutableStateOf(false) }
    var error by remember { mutableStateOf<String?>(null) }
    Box {
        OutlinedButton(onClick = { open = true }) { Text(if (state.replay == null) "Replay…" else "Replay: ${state.replayName}", maxLines = 1) }
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            DropdownMenuItem(text = { Text("Open .gpr file…") }, onClick = {
                open = false
                val d = FileDialog(null as Frame?, "Open replay", FileDialog.LOAD)
                d.setFilenameFilter { _, name -> name.endsWith(".gpr") }
                d.file = "*.gpr"
                d.isVisible = true
                val f = d.file
                if (f != null) {
                    val file = File(d.directory, f)
                    try { state.loadReplay(ReplayFiles.read(file), file.name); error = null } catch (e: Exception) { error = "${file.name}: ${e.message}" }
                    changed()
                }
            })
            ReplayFiles.builtIn().forEach { e ->
                DropdownMenuItem(text = { Text(e.label) }, onClick = { open = false; state.loadReplay(e.load(), e.label); changed() })
            }
        }
    }
    error?.let { Text(it, color = Color(0xFFFF6E6E), fontSize = 12.sp, maxLines = 1) }
}

@Composable
private fun InspectPanel(state: ViewerState, frameNo: Long) {
    Column(
        Modifier.width(300.dp).fillMaxHeight().padding(8.dp).verticalScroll(rememberScrollState()),
    ) {
        @Suppress("UNUSED_EXPRESSION") frameNo
        Text("stateHash ${java.lang.Long.toHexString(state.world.stateHash())}", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
        val p = state.player
        if (p != null) {
            val k = p.ticksPlayed - 1
            if (k >= 0) Text("recorded  ${java.lang.Long.toHexString(p.replay.hashAt(k))}", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
            state.divergentTick?.let { Text("first divergent tick $it", color = Color(0xFFFF6E6E), fontFamily = FontFamily.Monospace, fontSize = 12.sp) }
        }
        Text("contacts ${state.world.contactCount}   joints ${state.world.jointCount}", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
        val fp = state.world.forwardProgressX
        if (!fp.isNaN()) Text("forward progress x = ${"%.2f".format(fp)} m", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
        val ball = state.world.ball()
        if (ball.state != BallState.Dead || state.ballTrail.isNotEmpty()) {
            val mode = if (ball.state == BallState.Airborne) " " + ball.flightMode.name else ""
            Text("ball " + ball.state.name + mode, fontFamily = FontFamily.Monospace, fontSize = 12.sp)
            val speed = Math.sqrt(ball.vx * ball.vx + ball.vy * ball.vy + ball.vz * ball.vz)
            Text(
                "  (%.1f, %.1f) z %.2f m  %.1f m/s".format(ball.x, ball.y, ball.z, speed),
                fontFamily = FontFamily.Monospace, fontSize = 12.sp, maxLines = 1, softWrap = false,
            )
        }
        val wx = state.world.windX
        val wy = state.world.windY
        if (wx != 0.0 || wy != 0.0) Text("wind (%.1f, %.1f) m/s".format(wx, wy), fontFamily = FontFamily.Monospace, fontSize = 12.sp)
        Spacer(Modifier.width(1.dp).padding(4.dp))
        Text("events (newest first)", fontSize = 13.sp)
        state.eventLog.forEach { Text(it, fontFamily = FontFamily.Monospace, fontSize = 11.sp, maxLines = 1, softWrap = false, overflow = TextOverflow.Ellipsis) }
        Spacer(Modifier.width(1.dp).padding(4.dp))
        val id = state.selected
        if (id == null) {
            Text("Click a body to inspect it.", fontSize = 13.sp)
            return@Column
        }
        val b = state.world.body(id)
        fun f(v: Double) = "%.4f".format(v)
        val rows = listOf(
            "id" to id.value.toString(),
            "type" to b.type.name,
            "userData" to b.userData.toString(),
            "shape" to b.shape.toString(),
            "x" to f(b.x), "y" to f(b.y),
            "prevX" to f(b.prevX), "prevY" to f(b.prevY),
            "angle" to f(b.angle),
            "vx" to f(b.vx), "vy" to f(b.vy),
            "speed" to f(Math.hypot(b.vx, b.vy)),
            "angularVelocity" to f(b.angularVelocity),
            "mass" to f(b.mass), "invMass" to f(b.invMass), "inertia" to f(b.inertia),
            "linearDamping" to f(b.linearDamping), "angularDamping" to f(b.angularDamping),
            "radius" to f(b.radius),
            "bullet" to b.bullet.toString(), "awake" to b.awake.toString(),
            "categoryBits" to "0x" + Integer.toHexString(b.categoryBits),
            "maskBits" to "0x" + Integer.toHexString(b.maskBits),
            "material" to b.material.toString(),
            "sensor" to b.shape.sensor.toString(),
        )
        rows.forEach { (k, v) ->
            Row(Modifier.fillMaxWidth()) {
                Text(k, Modifier.weight(1f), fontSize = 12.sp, fontFamily = FontFamily.Monospace)
                Text(v, fontSize = 12.sp, fontFamily = FontFamily.Monospace)
            }
        }
    }
}
