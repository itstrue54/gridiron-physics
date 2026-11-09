package io.github.itstrue54.gridironphysics.bench

import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.WorldSnapshot
import io.github.itstrue54.gridironphysics.scenarios.RandomPlays
import io.github.itstrue54.gridironphysics.scenarios.Scenarios

/**
 * The 25-body benchmark world: [Scenarios.scrimmage] (25 players on locomotion, five blocks, a deep pass that bounces
 * loose), played for [PLAY_TICKS] ticks and then rewound to the snap with [rewind]. Rewinding (a snapshot restore)
 * allocates, so callers keep it outside anything they measure.
 */
class Scrimmage {
    val world: World = Scenarios.scrimmage.build()
    private val snap: WorldSnapshot = world.snapshot()

    /** Back to the snap. */
    fun rewind() = world.restore(snap)

    companion object {
        /** Ten seconds: the pass lands, bounces and rolls loose; the pile at the line keeps pushing. */
        const val PLAY_TICKS = 600
    }
}

/** Headless throughput, measured with the wall clock around `step()` only. */
object Throughput {
    class Result(val ticks: Long, val nanos: Long) {
        val ticksPerSecond: Double get() = ticks * 1e9 / nanos
        val microsPerTick: Double get() = nanos / 1e3 / ticks
    }

    /** Steps the scrimmage for about [seconds] of step time, rewinding every [Scrimmage.PLAY_TICKS] (not timed). */
    fun scrimmage(seconds: Double, s: Scrimmage = Scrimmage()): Result {
        val budget = (seconds * 1e9).toLong()
        var nanos = 0L
        var ticks = 0L
        while (nanos < budget) {
            s.rewind()
            val t0 = System.nanoTime()
            for (k in 0 until Scrimmage.PLAY_TICKS) s.world.step()
            nanos += System.nanoTime() - t0
            ticks += Scrimmage.PLAY_TICKS
        }
        return Result(ticks, nanos)
    }

    /** Plays the random-play harness from [first], timing only the engine's steps, for about [seconds] of step time. */
    fun randomPlays(seconds: Double, first: Int = 0): Result {
        val budget = (seconds * 1e9).toLong()
        val timer = StepTimer()
        var i = first
        while (timer.nanos < budget) { RandomPlays.play(i % RandomPlays.COUNT, stepper = timer); i++ }
        return Result(timer.ticks, timer.nanos)
    }

    /** Wall time of whole plays, the game layer included: how fast a game could simulate a season headless. */
    fun wholePlays(count: Int, first: Int = 0): Result {
        var ticks = 0L
        val t0 = System.nanoTime()
        for (i in first until first + count) ticks += RandomPlays.play(i % RandomPlays.COUNT).hashes.size
        return Result(ticks, System.nanoTime() - t0)
    }

    private class StepTimer : RandomPlays.Stepper {
        var nanos = 0L
        var ticks = 0L
        override fun step(world: World) {
            val t0 = System.nanoTime()
            world.step()
            nanos += System.nanoTime() - t0
            ticks++
        }
    }
}

/** `./gradlew :physics-bench:ticksPerSecond`: warms up, then prints the best of five runs of each workload. */
object TicksPerSecond {
    @JvmStatic fun main(args: Array<String>) {
        val s = Scrimmage()
        Throughput.scrimmage(3.0, s)
        Throughput.randomPlays(2.0)
        var scrim = Throughput.scrimmage(1.0, s)
        var plays = Throughput.randomPlays(1.0)
        repeat(4) {
            val a = Throughput.scrimmage(1.0, s); if (a.ticksPerSecond > scrim.ticksPerSecond) scrim = a
            val b = Throughput.randomPlays(1.0, 300); if (b.ticksPerSecond > plays.ticksPerSecond) plays = b
        }
        val whole = Throughput.wholePlays(1000)
        println("25-player scrimmage, step only : %,.0f ticks/s (%.1f us/step)".format(scrim.ticksPerSecond, scrim.microsPerTick))
        println("random plays, step only        : %,.0f ticks/s (%.1f us/step)".format(plays.ticksPerSecond, plays.microsPerTick))
        println("1,000 whole plays, game layer   : %,.0f ticks/s (%.2f s for %,d ticks)".format(whole.ticksPerSecond, whole.nanos / 1e9, whole.ticks))
        println("gate: >= 20,000 ticks/s headless desktop")
    }
}
