package io.github.itstrue54.gridironphysics

import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/** Fails the build if physics-core's main sources use an API that breaks determinism or portability. */
class DeterminismLintTest {
    private val banned = listOf(
        Regex("""(?<![A-Za-z0-9_])Math\."""), // java.lang.Math; StrictMath and DMath are fine
        Regex("""kotlin\.math"""),
        Regex("""java\.util\.Random"""),
        Regex("""kotlin\.random"""),
        Regex("""HashMap|HashSet|hashMapOf|hashSetOf|mutableMapOf|mutableSetOf"""),
        Regex("""System\.(nanoTime|currentTimeMillis)"""),
        Regex("""\bThread\b|kotlinx\.coroutines"""),
        Regex("""java\.io\.|java\.nio\.file"""),
        Regex("""^import android\.""", RegexOption.MULTILINE),
    )

    @Test fun mainSourcesUseNoBannedApis() {
        val root = File("src/main/kotlin")
        assertTrue(root.isDirectory, "run from the physics-core module dir; missing ${root.absolutePath}")
        val hits = mutableListOf<String>()
        root.walkTopDown().filter { it.isFile && it.extension == "kt" }.forEach { f ->
            f.readLines().forEachIndexed { i, line ->
                banned.forEach { re -> re.find(line)?.let { hits += "${f.path}:${i + 1}: ${it.value}  |  ${line.trim()}" } }
            }
        }
        assertTrue(hits.isEmpty(), "Banned APIs in physics-core:\n" + hits.joinToString("\n"))
    }
}
