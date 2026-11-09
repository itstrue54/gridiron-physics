package io.github.itstrue54.gridironphysics.debug

import io.github.itstrue54.gridironphysics.scenarios.RandomPlays
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

class ReplayViewerTest {
    private fun playToEnd(s: ViewerState) {
        var guard = 0
        while (!s.player!!.finished && guard++ < 100_000) {
            s.paused = false
            s.frame(1.0 / 60)
        }
    }

    @Test fun aBuiltInReplayPlaysToTheEndVerified() {
        val e = ReplayFiles.builtIn()[1]
        val s = ViewerState()
        s.loadReplay(e.load(), e.label)
        assertEquals(s.replay!!.startTick, s.world.tick)
        playToEnd(s)
        assertNull(s.divergentTick, s.player!!.divergence)
        assertEquals(s.replay!!.endTick, s.world.tick)
        assertTrue(s.paused, "playback stops at the end")
    }

    @Test fun theCorruptedDemoIsFlaggedAtTick120AndPausesThere() {
        val all = ReplayFiles.builtIn()
        val e = all.last()
        val s = ViewerState()
        s.loadReplay(e.load(), e.label)
        var guard = 0
        while (!s.paused && guard++ < 10_000) s.frame(1.0 / 60)
        assertEquals(120L, s.divergentTick)
        assertEquals(120L, s.world.tick, "the viewer stops on the divergent tick")
        // Play carries on past it; the flag stays.
        playToEnd(s)
        assertEquals(120L, s.divergentTick)
    }

    @Test fun replayFilesRoundTripAndStepWorks() {
        val r = RandomPlays.play(4, record = true).replay!!
        val dir = Files.createTempDirectory("gpr").toFile()
        val f = File(dir, "play.gpr")
        ReplayFiles.write(f, r)
        val back = ReplayFiles.read(f)
        assertEquals(r.tickCount, back.tickCount)
        val s = ViewerState()
        s.loadReplay(back, f.name)
        s.paused = true
        s.stepOnce(); s.stepOnce()
        assertEquals(back.startTick + 2, s.world.tick)
        assertEquals(2, s.player!!.ticksPlayed)
        s.reset()
        assertEquals(back.startTick, s.world.tick)
        // Choosing a scenario leaves replay mode.
        s.scenario = s.scenario
        assertNull(s.replay)
        assertFalse(s.player != null)
        dir.deleteRecursively()
    }
}
