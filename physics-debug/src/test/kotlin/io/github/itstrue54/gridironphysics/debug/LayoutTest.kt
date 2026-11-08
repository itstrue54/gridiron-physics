package io.github.itstrue54.gridironphysics.debug

import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.darkColorScheme
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.ImageComposeScene
import androidx.compose.ui.Modifier
import androidx.compose.ui.use
import org.jetbrains.skia.Bitmap
import org.jetbrains.skia.Image
import kotlin.test.Test
import kotlin.test.assertTrue

/** The field must stay visible however narrow the window is: the top bar never pushes it off screen. */
class LayoutTest {
    @OptIn(ExperimentalComposeUiApi::class)
    private fun render(w: Int, h: Int): Bitmap = ImageComposeScene(width = w, height = h).use { scene ->
        scene.setContent { MaterialTheme(colorScheme = darkColorScheme()) { Surface(Modifier.fillMaxSize()) { ViewerApp() } } }
        var t = 0L
        repeat(5) { scene.render(t); t += 16_000_000L }
        Bitmap.makeFromImage(scene.render(t))
    }

    /** Share of pixels in the lower 60% of the window that are field green. */
    private fun greenShare(b: Bitmap): Double {
        var green = 0; var total = 0
        for (y in (b.height * 0.4).toInt() until b.height step 4) for (x in 0 until (b.width * 0.7).toInt() step 4) {
            val c = b.getColor(x, y)
            val r = (c shr 16) and 0xFF; val g = (c shr 8) and 0xFF; val bl = c and 0xFF
            if (g > 80 && g > r + 30 && g > bl + 30) green++
            total++
        }
        return green.toDouble() / total
    }

    @Test fun fieldVisibleAtWideWindow() = assertTrue(greenShare(render(1500, 860)) > 0.5)

    @Test fun fieldVisibleAtTheUsersWindowSize() = assertTrue(greenShare(render(1352, 723)) > 0.5, "field hidden at 1352x723")

    @Test fun fieldVisibleAtSmallWindow() = assertTrue(greenShare(render(1000, 650)) > 0.5, "field hidden at 1000x650")
}
