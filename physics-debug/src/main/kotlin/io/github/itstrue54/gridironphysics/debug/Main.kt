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
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.application
import androidx.compose.ui.window.rememberWindowState
import io.github.itstrue54.gridironphysics.scenarios.Scenario
import io.github.itstrue54.gridironphysics.scenarios.Scenarios

fun main() = application {
    Window(
        onCloseRequest = ::exitApplication,
        title = "gridiron-physics viewer",
        state = rememberWindowState(width = 1500.dp, height = 860.dp),
    ) {
        MaterialTheme(colorScheme = darkColorScheme()) {
            Surface(Modifier.fillMaxSize()) { ViewerApp() }
        }
    }
}

@OptIn(ExperimentalComposeUiApi::class)
@Composable
fun ViewerApp(initial: Scenario = Scenarios.all.first(), showColliders: Boolean = false) {
    val state = remember { ViewerState(initial).also { it.showColliders = showColliders } }
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
        TopBar(state, frameNo) { frameNo++ }
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
                drawBodies(t, state)
                drawContacts(t, state, textMeasurer)
            }
            InspectPanel(state, frameNo)
        }
    }
}

@Composable
private fun TopBar(state: ViewerState, frameNo: Long, changed: () -> Unit) {
    var menuOpen by remember { mutableStateOf(false) }
    Row(
        Modifier.fillMaxWidth().padding(8.dp),
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
            OutlinedButton(onClick = { menuOpen = true }) { Text("Scenario: ${state.scenario.name}") }
            DropdownMenu(expanded = menuOpen, onDismissRequest = { menuOpen = false }) {
                Scenarios.all.forEach { sc ->
                    DropdownMenuItem(text = { Text(sc.name) }, onClick = { state.scenario = sc; menuOpen = false; changed() })
                }
            }
        }
        OutlinedButton(onClick = { state.reset(); changed() }) { Text("Reset") }
        FilterChip(selected = state.follow, onClick = { state.follow = !state.follow; changed() }, label = { Text("Follow") })
        OutlinedButton(onClick = { state.showFullField(); changed() }) { Text("Full field") }
        FilterChip(selected = state.showColliders, onClick = { state.showColliders = !state.showColliders; changed() }, label = { Text("Colliders") })
        Spacer(Modifier.width(16.dp))
        Text("tick ${state.world.tick}   t = ${"%.2f".format(state.world.tick / 60.0)} s   (frame $frameNo)", fontFamily = FontFamily.Monospace)
        Spacer(Modifier.weight(1f))
        Text(state.scenario.description + "   (wheel: zoom, drag: pan)", fontSize = 12.sp)
    }
}

@Composable
private fun InspectPanel(state: ViewerState, frameNo: Long) {
    Column(
        Modifier.width(300.dp).fillMaxHeight().padding(8.dp).verticalScroll(rememberScrollState()),
    ) {
        @Suppress("UNUSED_EXPRESSION") frameNo
        Text("stateHash ${java.lang.Long.toHexString(state.world.stateHash())}", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
        Text("contacts ${state.world.contactCount}", fontFamily = FontFamily.Monospace, fontSize = 12.sp)
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
