package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.Replay
import io.github.itstrue54.gridironphysics.scenarios.DeterminismSuite
import java.io.File

/** `.gpr` file access for the viewer (physics-core only encodes and decodes bytes). */
object ReplayFiles {
    fun read(file: File): Replay = Replay.decode(file.readBytes())

    fun write(file: File, replay: Replay) {
        file.parentFile?.mkdirs()
        file.writeBytes(replay.encode())
    }

    /** A replay the viewer offers in its menu. */
    class Entry(val label: String, val load: () -> Replay)

    /** The committed golden replays, plus one with a hash corrupted on purpose, to show the divergence flag. */
    fun builtIn(): List<Entry> {
        val out = ArrayList<Entry>()
        for (i in 0 until DeterminismSuite.GOLDEN_REPLAY_COUNT) {
            val name = DeterminismSuite.goldenReplayName(i)
            out += Entry("Harness play $i (${name.substringAfterLast('/')})") { Replay.decode(resource(name)) }
        }
        out += Entry("Harness play 0, hash corrupted at tick 120 (divergence demo)") {
            val r = Replay.decode(resource(DeterminismSuite.goldenReplayName(0)))
            Replay.decode(corruptHash(r.encode(), r, (120 - r.startTick - 1).toInt()))
        }
        return out
    }

    private fun resource(name: String): ByteArray = checkNotNull(DeterminismSuite.resource(name)) { "missing resource $name" }

    /** [bytes] (the encoding of [r]) with recorded hash [k] (the state after tick startTick + k + 1) changed. */
    fun corruptHash(bytes: ByteArray, r: Replay, k: Int): ByteArray {
        require(k in 0 until r.tickCount) { "hash $k out of range 0 until ${r.tickCount}" }
        val out = bytes.copyOf()
        val at = out.size - 8 * (r.tickCount - k)
        out[at] = (out[at].toInt() xor 1).toByte()
        return out
    }
}
