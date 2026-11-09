package io.github.itstrue54.gridironphysics.debug

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.darkColorScheme
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.ImageComposeScene
import androidx.compose.ui.Modifier
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.ui.use
import org.jetbrains.skia.EncodedImageFormat
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import java.io.File

/**
 * Headless render of the viewer to a PNG, for docs and CI artifacts.
 * Usage: ./gradlew :physics-debug:snapshot -Pout=build/viewer.png -Pseconds=1.5 [-Pscenario="Contact lab"] [-Pcolliders=true]
 * [-Preplay=N] (the viewer's Nth built-in replay instead of a scenario)
 */
@OptIn(ExperimentalComposeUiApi::class)
fun main(args: Array<String>) {
    val out = File(args.getOrElse(0) { "build/viewer.png" })
    val seconds = args.getOrElse(1) { "1.5" }.toDouble()
    val scenario = args.getOrNull(2)?.let { name -> Scenarios.all.first { it.name == name } } ?: Scenarios.all.first()
    val colliders = args.getOrNull(3) == "true"
    val width = args.getOrNull(4)?.toInt() ?: 1500
    val height = args.getOrNull(5)?.toInt() ?: 860
    val replayEntry = args.getOrNull(6)?.toInt() ?: -1
    ImageComposeScene(width = width, height = height).use { scene ->
        scene.setContent {
            MaterialTheme(colorScheme = darkColorScheme()) { Surface(Modifier.fillMaxSize()) { ViewerApp(scenario, colliders, replayEntry = replayEntry) } }
        }
        val frameNs = 1_000_000_000L / 60
        var t = 0L
        val frames = (seconds * 60).toInt() + 2
        var image = scene.render(t)
        repeat(frames) { t += frameNs; image = scene.render(t) }
        out.parentFile?.mkdirs()
        out.writeBytes(image.encodeToData(EncodedImageFormat.PNG)!!.bytes)
        println("wrote ${out.absolutePath}")
    }
}
