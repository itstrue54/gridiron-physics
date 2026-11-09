package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.World
import java.lang.management.ManagementFactory
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * M8 gate: Performance, 0 allocations per step. Counts the bytes the test thread allocates inside `World.step()`
 * only (the game layer around it, world setup and joint creation are outside the window), across the 1,000-play
 * determinism harness (locomotion, contacts, every joint type, ball flight, bounces, CCD, events, recording) and
 * every scenario.
 *
 * The JIT may allocate a few hundred bytes once in a window (linking, deoptimization), so after a long warm-up the
 * measurement runs up to 3 times and one attempt must be exactly 0.
 */
class StepAllocationTest {
    private val mx = ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean
    private val tid = Thread.currentThread().id
    private var sink = IntArray(0)

    /** Adds up the bytes allocated inside each step. */
    private inner class Meter : RandomPlays.Stepper {
        var bytes = 0L
        var steps = 0L
        var worst = 0L
        override fun step(world: World) {
            val m = mx!!
            val before = m.getThreadAllocatedBytes(tid)
            world.step()
            val d = m.getThreadAllocatedBytes(tid) - before
            bytes += d; steps++
            if (d > worst) worst = d
        }
    }

    private fun supported(): Boolean = mx != null && mx.isThreadAllocatedMemorySupported.also { if (it) mx.isThreadAllocatedMemoryEnabled = true }

    @Test fun theMeterItselfDoesNotAllocate() {
        if (!supported()) return
        val m = mx!!
        var total = 0L
        repeat(100_000) { val b = m.getThreadAllocatedBytes(tid); total += m.getThreadAllocatedBytes(tid) - b }
        val attempts = LongArray(3)
        for (a in 0 until 3) {
            var t = 0L
            repeat(100_000) { val b = m.getThreadAllocatedBytes(tid); t += m.getThreadAllocatedBytes(tid) - b }
            attempts[a] = t
            if (t == 0L) break
        }
        assertTrue(attempts.any { it == 0L }, "an empty window must read 0 bytes: ${attempts.toList()} (warm-up $total)")
    }

    @Test fun theMeterSeesAnAllocation() {
        if (!supported()) return
        val m = mx!!
        val b = m.getThreadAllocatedBytes(tid)
        sink = IntArray(64 + sink.size % 2)
        val d = m.getThreadAllocatedBytes(tid) - b
        assertTrue(d >= 64 * 4, "a 64-int array must be counted, read $d bytes")
    }

    @Test fun stepAllocatesNothingAcross1000RandomPlays() {
        if (!supported()) return
        val warm = Meter()
        for (i in 0 until 300) RandomPlays.play(i, stepper = warm) // warm-up: the JIT compiles every path
        val attempts = ArrayList<String>()
        for (attempt in 0 until 3) {
            val m = Meter()
            for (i in 0 until RandomPlays.COUNT) RandomPlays.play(i, record = false, stepper = m)
            attempts += "${m.bytes} bytes in ${m.steps} steps (worst step ${m.worst})"
            if (m.bytes == 0L) { println("StepAllocationTest: $attempts"); return }
        }
        throw AssertionError("step() allocates across the harness, every attempt: $attempts")
    }

    @Test fun stepAllocatesNothingInAnyScenario() {
        if (!supported()) return
        val warm = Meter()
        repeat(3) { for (s in Scenarios.all) { val w = s.build(); repeat(900) { warm.step(w) } } }
        val attempts = ArrayList<String>()
        for (attempt in 0 until 3) {
            val per = LinkedHashMap<String, Long>()
            for (s in Scenarios.all) {
                val w = s.build() // setup stays outside the window
                val m = Meter()
                repeat(900) { m.step(w) } // 15 s: every scenario runs to its end
                per[s.name] = m.bytes
            }
            attempts += per.toString()
            if (per.values.all { it == 0L }) { println("StepAllocationTest: ${Scenarios.all.size} scenarios x 900 steps, $attempts"); return }
        }
        throw AssertionError("step() allocates in a scenario, every attempt:\n" + attempts.joinToString("\n"))
    }

    @Test fun theScrimmageHas25DynamicPlayers() {
        val w = Scenarios.scrimmage.build()
        val ids = IntArray(64)
        val n = w.bodyIds(ids)
        var dynamic = 0
        for (k in 0 until n) if (w.body(io.github.itstrue54.gridironphysics.BodyId(ids[k])).type == io.github.itstrue54.gridironphysics.BodyType.Dynamic) dynamic++
        assertEquals(25, dynamic)
    }
}
