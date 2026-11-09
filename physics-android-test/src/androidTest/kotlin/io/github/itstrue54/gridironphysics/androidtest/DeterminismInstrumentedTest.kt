package io.github.itstrue54.gridironphysics.androidtest

import androidx.test.ext.junit.runners.AndroidJUnit4
import io.github.itstrue54.gridironphysics.scenarios.DeterminismSuite
import io.github.itstrue54.gridironphysics.scenarios.RandomPlays
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The M7 determinism gates on ART: the same 1,000 seeded plays as the desktop tests. Cross-device passes when every
 * play's hash sequence matches the golden digests made on the desktop, and the committed `.gpr` replays play back clean.
 */
@RunWith(AndroidJUnit4::class)
class DeterminismInstrumentedTest {
    @Test fun crossDevice1000PlaysMatchTheDesktopGoldenHashes() {
        val fails = DeterminismSuite.crossDevice(0 until RandomPlays.COUNT)
        assertTrue("${fails.size} failures:\n" + fails.take(20).joinToString("\n"), fails.isEmpty())
    }

    @Test fun sameDevice1000PlaysRunTwiceAndReplayIdentically() {
        val fails = DeterminismSuite.sameDevice(0 until RandomPlays.COUNT)
        assertTrue("${fails.size} failures:\n" + fails.take(20).joinToString("\n"), fails.isEmpty())
    }

    @Test fun goldenReplaysPlayBackClean() {
        val fails = DeterminismSuite.goldenReplays()
        assertTrue(fails.joinToString("\n"), fails.isEmpty())
    }
}
