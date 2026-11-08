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
import java.io.File

/**
 * Headless render of the viewer to a PNG, for docs and CI artifacts.
 * Usage: ./gradlew :physics-debug:snapshot -Pout=build/viewer.png -Pseconds=1.5
 */
@OptIn(ExperimentalComposeUiApi::class)
fun main(args: Array<String>) {
    val out = File(args.getOrElse(0) { "build/viewer.png" })
    val seconds = args.getOrElse(1) { "1.5" }.toDouble()
    ImageComposeScene(width = 1500, height = 860).use { scene ->
        scene.setContent {
            MaterialTheme(colorScheme = darkColorScheme()) { Surface(Modifier.fillMaxSize()) { ViewerApp() } }
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
