package io.github.itstrue54.gridironphysics.scenarios

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** M7 gates: Determinism, same device (1,000 plays) and Determinism, cross-device (golden digests from another machine). */
class DeterminismHarnessTest {
    @Test fun sameDevice1000PlaysRunTwiceAndReplayIdentically() {
        val fails = DeterminismSuite.sameDevice(0 until RandomPlays.COUNT)
        assertTrue(fails.isEmpty(), "${fails.size} failures:\n" + fails.take(20).joinToString("\n"))
    }

    @Test fun crossDevice1000PlaysMatchTheGoldenHashes() {
        val fails = DeterminismSuite.crossDevice(0 until RandomPlays.COUNT)
        assertTrue(fails.isEmpty(), "${fails.size} failures:\n" + fails.take(20).joinToString("\n"))
    }

    @Test fun theHarnessExercisesTheEngine() {
        val total = IntArray(RandomPlays.S_STATS)
        var ticks = 0
        for (i in 0 until 200) {
            val r = RandomPlays.play(i)
            ticks += r.hashes.size
            for (k in total.indices) total[k] += r.stats[k]
        }
        val names = listOf("grapples", "blocks", "passes", "catches", "kicks", "bounces", "pickups", "downs", "hits", "kick results",
            "breaks", "created", "destroyed", "fumbles", "deflects")
        val summary = names.indices.joinToString { "${names[it]} ${total[it]}" }
        println("200 plays, $ticks ticks: $summary")
        for (k in total.indices) assertTrue(total[k] >= 5, "the harness must produce ${names[k]}: $summary")
    }

    @Test fun goldenFormatRoundTrips() {
        val text = DeterminismSuite.formatGolden(longArrayOf(-1L, 5L), intArrayOf(240, 480))
        val g = DeterminismSuite.parseGolden(text)
        assertEquals(-1L, g.digests[0]); assertEquals(5L, g.digests[1]); assertEquals(480, g.ticks[1])
    }
}
