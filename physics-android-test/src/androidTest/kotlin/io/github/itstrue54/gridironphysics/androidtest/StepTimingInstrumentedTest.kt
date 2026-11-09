package io.github.itstrue54.gridironphysics.androidtest

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import io.github.itstrue54.gridironphysics.scenarios.Scenarios
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Spec row "Performance", device part: step() time of the 25-player scrimmage on ART. The budget (<= 0.5 ms per step)
 * is for a mid-range phone; an emulator on a CI runner is neither, so this only reports the time (logcat tag
 * GridironBench, printed by the CI job) and fails only if a step takes more than 10x the budget, which would mean
 * something pathological rather than a slow host.
 */
@RunWith(AndroidJUnit4::class)
class StepTimingInstrumentedTest {
    @Test fun scrimmageStepTime() {
        val w = Scenarios.scrimmage.build()
        val snap = w.snapshot()
        fun play(): Long {
            w.restore(snap)
            val t0 = System.nanoTime()
            for (k in 0 until TICKS) w.step()
            return System.nanoTime() - t0
        }
        repeat(20) { play() } // warm-up: ART's JIT compiles the hot paths
        var best = Long.MAX_VALUE
        var total = 0L
        repeat(RUNS) { val t = play(); total += t; if (t < best) best = t }
        val bestMs = best / 1e6 / TICKS
        val meanMs = total / 1e6 / (TICKS * RUNS)
        Log.i(TAG, "25-player scrimmage on ART: mean %.4f ms/step, best run %.4f ms/step (%d runs x %d ticks); budget 0.5 ms on a mid-range phone"
            .format(meanMs, bestMs, RUNS, TICKS))
        assertTrue("step takes $meanMs ms on average", meanMs < 5.0)
    }

    private companion object {
        const val TAG = "GridironBench"
        const val TICKS = 600
        const val RUNS = 20
    }
}
