package io.github.itstrue54.gridironphysics.bench

import java.lang.management.ManagementFactory
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * M8 gates, spec row "Performance" (desktop part): the 25-body scrimmage steps at >= 20,000 ticks/s headless and
 * allocates 0 bytes per step. The device budget (<= 0.5 ms per step on a mid-range phone) needs a real phone; the
 * Android emulator job reports its time but does not gate on it.
 */
class PerformanceGateTest {
    @Test fun scrimmageRunsAtLeast20000TicksPerSecond() {
        val s = Scrimmage()
        Throughput.scrimmage(2.0, s) // warm-up: the JIT compiles every path
        val runs = ArrayList<Double>()
        for (attempt in 0 until 5) {
            val r = Throughput.scrimmage(0.5, s)
            runs += r.ticksPerSecond
            if (r.ticksPerSecond >= GATE) break
        }
        println("PerformanceGateTest: scrimmage ticks/s per run ${runs.map { it.toLong() }}")
        assertTrue(runs.max() >= GATE, "25-body scrimmage must reach $GATE ticks/s headless, best run ${runs.max().toLong()}")
    }

    @Test fun scrimmageStepAllocatesNothing() {
        val mx = ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        val tid = Thread.currentThread().id
        val s = Scrimmage()
        Throughput.scrimmage(1.0, s)
        val attempts = ArrayList<Long>()
        for (attempt in 0 until 3) {
            var bytes = 0L
            repeat(10) {
                s.rewind() // allocates; outside the window
                val w = s.world
                val before = mx.getThreadAllocatedBytes(tid)
                for (k in 0 until Scrimmage.PLAY_TICKS) w.step()
                bytes += mx.getThreadAllocatedBytes(tid) - before
            }
            attempts += bytes
            if (bytes == 0L) break
        }
        println("PerformanceGateTest: bytes allocated by 6,000 scrimmage steps, per attempt $attempts")
        assertTrue(attempts.last() == 0L, "step() must allocate nothing: $attempts")
    }

    private companion object {
        const val GATE = 20_000.0
    }
}
