package io.github.itstrue54.gridironphysics.bench

import io.github.itstrue54.gridironphysics.scenarios.RandomPlays
import kotlinx.benchmark.Benchmark
import kotlinx.benchmark.Blackhole
import kotlinx.benchmark.Scope
import kotlinx.benchmark.Setup
import kotlinx.benchmark.State
import org.openjdk.jmh.annotations.Level
import org.openjdk.jmh.annotations.OperationsPerInvocation

/**
 * Spec row "Performance": 25 dynamic bodies. One invocation plays the scrimmage from the snap for
 * [Scrimmage.PLAY_TICKS] ticks; the score is per step (`@OperationsPerInvocation`). The rewind runs in an
 * invocation-level setup, outside the timing (each invocation is several milliseconds, so the setup overhead is noise).
 * With the GC profiler, gc.alloc.rate.norm still counts the rewind's allocation spread over the 600 steps; the
 * zero-allocation gate is the test in this module, which measures step() alone.
 */
@State(Scope.Benchmark)
class StepBenchmark {
    private lateinit var s: Scrimmage

    @Setup
    fun build() { s = Scrimmage() }

    @Setup(Level.Invocation)
    fun rewind() = s.rewind()

    @Benchmark
    @OperationsPerInvocation(Scrimmage.PLAY_TICKS)
    fun scrimmageStep(bh: Blackhole) {
        val w = s.world
        for (k in 0 until Scrimmage.PLAY_TICKS) w.step()
        bh.consume(w.stateHash())
    }
}

/** A whole random play, game layer included (the harness's decisions, joint creation, recording), per tick. */
@State(Scope.Benchmark)
class PlayBenchmark {
    private var index = 0

    @Benchmark
    fun randomPlay(bh: Blackhole) {
        val r = RandomPlays.play(index)
        index = (index + 1) % RandomPlays.COUNT
        bh.consume(r.hashes.size)
    }
}
