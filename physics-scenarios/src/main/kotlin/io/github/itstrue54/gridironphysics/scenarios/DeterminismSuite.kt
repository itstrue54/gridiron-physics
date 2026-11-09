package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.Replay
import io.github.itstrue54.gridironphysics.ReplayPlayer
import java.io.File

/**
 * The determinism gates, as plain functions that return their failures, so the desktop JUnit tests and the Android
 * instrumented tests run exactly the same checks.
 *
 * - [sameDevice]: every play run twice from scratch gives identical hashes every tick, and its recorded replay plays
 *   back with every hash matching.
 * - [crossDevice]: every play's hash sequence matches the golden digests committed in `golden/plays.txt` (made on one
 *   machine by [GenerateGolden]), and the committed `.gpr` replays play back clean. Run on Windows, macOS, Linux and
 *   Android, this proves the hash sequences are identical across devices.
 */
object DeterminismSuite {
    const val GOLDEN_PLAYS = "golden/plays.txt"
    const val GOLDEN_REPLAY_COUNT = 8
    const val HEADER = "# gridiron-physics determinism golden v1: play index, digest of its per-tick state hashes, ticks"

    fun goldenReplayName(i: Int): String = "golden/replay-" + i.toString().padStart(3, '0') + ".gpr"

    /** A classpath resource (desktop jar or Android APK), or null. */
    fun resource(name: String): ByteArray? =
        DeterminismSuite::class.java.classLoader?.getResourceAsStream(name)?.use { it.readBytes() }

    /** Golden digest and tick count per play index. */
    class Golden(val digests: LongArray, val ticks: IntArray)

    fun parseGolden(text: String): Golden {
        val lines = text.lines().map { it.trim() }.filter { it.isNotEmpty() && !it.startsWith("#") }
        val d = LongArray(lines.size); val t = IntArray(lines.size)
        for ((k, line) in lines.withIndex()) {
            val p = line.split(" ")
            require(p.size == 3 && p[0].toInt() == k) { "bad golden line $k: $line" }
            d[k] = java.lang.Long.parseUnsignedLong(p[1], 16); t[k] = p[2].toInt()
        }
        return Golden(d, t)
    }

    fun loadGolden(): Golden = parseGolden(String(checkNotNull(resource(GOLDEN_PLAYS)) { "missing resource $GOLDEN_PLAYS" }, Charsets.UTF_8))

    fun formatGolden(digests: LongArray, ticks: IntArray): String = buildString {
        append(HEADER).append('\n')
        for (i in digests.indices) append(i).append(' ').append(java.lang.Long.toHexString(digests[i]).padStart(16, '0')).append(' ').append(ticks[i]).append('\n')
    }

    /** Plays [range] twice each (the first recorded) and replays each recording. Returns the failures. */
    fun sameDevice(range: IntRange): List<String> {
        val fails = ArrayList<String>()
        for (i in range) {
            val a = RandomPlays.play(i, record = true)
            val b = RandomPlays.play(i)
            if (!a.hashes.contentEquals(b.hashes)) {
                val k = a.hashes.indices.first { a.hashes[it] != b.hashes[it] }
                fails += "play $i: second run differs at tick ${k + 1}"
                continue
            }
            val replay = Replay.decode(a.replay!!.encode())
            val p = ReplayPlayer(replay)
            val div = p.run()
            if (div != ReplayPlayer.NO_DIVERGENCE) fails += "play $i: replay diverges at tick $div: ${p.divergence}"
            val off = a.hashes.size - replay.tickCount
            for (k in 0 until replay.tickCount) {
                if (replay.hashAt(k) != a.hashes[off + k]) { fails += "play $i: recorded hash $k differs from the run"; break }
            }
        }
        return fails
    }

    /** Compares [range]'s hash sequences with the golden digests, and plays the golden replays. Returns the failures. */
    fun crossDevice(range: IntRange): List<String> {
        val g = loadGolden()
        val fails = ArrayList<String>()
        for (i in range) {
            if (i >= g.digests.size) { fails += "play $i has no golden digest"; continue }
            val h = RandomPlays.play(i).hashes
            if (h.size != g.ticks[i]) fails += "play $i: ${h.size} ticks, golden ${g.ticks[i]}"
            else if (RandomPlays.digest(h) != g.digests[i]) fails += "play $i: hash sequence differs from the golden one"
        }
        fails += goldenReplays()
        return fails
    }

    /** Each committed `.gpr` replays without divergence. */
    fun goldenReplays(): List<String> {
        val fails = ArrayList<String>()
        for (i in 0 until GOLDEN_REPLAY_COUNT) {
            val bytes = resource(goldenReplayName(i))
            if (bytes == null) { fails += "missing ${goldenReplayName(i)}"; continue }
            val p = ReplayPlayer(Replay.decode(bytes))
            val div = p.run()
            if (div != ReplayPlayer.NO_DIVERGENCE) fails += "${goldenReplayName(i)} diverges at tick $div: ${p.divergence}"
        }
        return fails
    }
}

/**
 * Writes the golden data into a resources directory (argument 1): `./gradlew :physics-scenarios:generateGolden`.
 * Run it only when the engine's behaviour is meant to change, and commit the result.
 */
object GenerateGolden {
    @JvmStatic fun main(args: Array<String>) {
        val dir = File(args.getOrElse(0) { "src/main/resources" }, "golden")
        dir.mkdirs()
        val d = LongArray(RandomPlays.COUNT); val t = IntArray(RandomPlays.COUNT)
        for (i in 0 until RandomPlays.COUNT) {
            val r = RandomPlays.play(i, record = i < DeterminismSuite.GOLDEN_REPLAY_COUNT)
            d[i] = RandomPlays.digest(r.hashes); t[i] = r.hashes.size
            if (r.replay != null) File(dir.parentFile, DeterminismSuite.goldenReplayName(i)).writeBytes(r.replay.encode())
        }
        File(dir, "plays.txt").writeText(DeterminismSuite.formatGolden(d, t))
        println("wrote ${RandomPlays.COUNT} digests and ${DeterminismSuite.GOLDEN_REPLAY_COUNT} replays to ${dir.absolutePath}")
    }
}
