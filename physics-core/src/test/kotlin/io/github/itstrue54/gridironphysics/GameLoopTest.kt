package io.github.itstrue54.gridironphysics

import java.lang.management.ManagementFactory
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNotEquals
import kotlin.test.assertTrue

/**
 * M9: what a game needs to drive the engine every tick without frame timing or allocations leaking in. A tick
 * listener on the world (so [World.advance] can carry a game that decides between ticks), joints and lying shapes
 * that cost nothing to make once the world exists, and a ball carrier read and set without boxing.
 */
class GameLoopTest {
    private val f = FieldConfig.PRO

    private fun world(config: WorldConfig = WorldConfig()) = World(config, f, 1L)

    private fun ghost(w: World, x: Double, y: Double = 24.0, vx: Double = 0.0): BodyId = w.createBody(
        BodyDef(x = x, y = y, vx = vx, mass = 100.0, shape = ShapeDef.Circle(0.34), maskBits = Category.STATIC),
    )

    private val runner = AthleteParams(maxSpeed = 9.0, maxAccel = 7.0, maxDecel = 8.0, turnRateAtRest = 12.0, turnRateAtMax = 2.0, traction = 0.9)

    // ------------------------------------------------------------ tick listener

    @Test fun theListenerRunsAroundEveryTick() {
        val w = world()
        val seen = ArrayList<String>()
        w.tickListener = object : TickListener {
            override fun beforeTick(world: World) { seen += "before ${world.tick}" }
            override fun afterTick(world: World) { seen += "after ${world.tick}" }
        }
        w.step()
        w.advance(2.0 / 60.0)
        assertEquals(listOf("before 0", "after 1", "before 1", "after 2", "before 2", "after 3"), seen)
        w.tickListener = null
        w.step()
        assertEquals(6, seen.size)
    }

    @Test fun afterTickSeesThatTicksEvents() {
        val w = world()
        val a = ghost(w, 30.0, vx = 3.0)
        w.setAthlete(a, runner)
        w.setIntent(a, LocomotionIntent.velocity(3.0, 0.0))
        w.setBallCarrier(a)
        val b = ghost(w, 31.0)
        var grabbedAt = -1L
        var downs = 0
        w.tickListener = object : TickListener {
            override fun beforeTick(world: World) {
                if (world.tick == 5L) { world.createGrapple(b, a, GrappleDef(breakForce = 1e9, dragForce = 2000.0)); grabbedAt = world.tick }
                if (world.tick == 30L) world.setStance(a, Stance.Down)
            }
            override fun afterTick(world: World) {
                val e = world.events()
                while (e.next()) if (e.type == EventType.Down && e.ballCarrier) downs++
            }
        }
        repeat(40) { w.advance(1.0 / 60.0) }
        assertEquals(5L, grabbedAt)
        assertEquals(1, downs)
    }

    /** A game that decides from the world each tick ends up in the same state however the frames fall. */
    @Test fun decisionsOnTicksAreIndependentOfFrameTiming() {
        fun play(frames: DoubleArray?): Long {
            val w = world()
            val c = ghost(w, 30.0)
            w.setAthlete(c, runner)
            val chaser = ghost(w, 25.0, y = 20.0)
            w.setAthlete(chaser, runner)
            w.tickListener = object : TickListener {
                override fun beforeTick(world: World) {
                    // Zigzag every 20 ticks; the chaser heads for where the carrier is now.
                    val sign = if ((world.tick / 20) % 2 == 0L) 1.0 else -1.0
                    world.setIntent(c, LocomotionIntent.velocity(6.0, 4.0 * sign))
                    val v = world.body(c)
                    world.setIntent(chaser, LocomotionIntent.point(v.x, v.y))
                }
            }
            if (frames == null) repeat(240) { w.step() }
            else { var k = 0; while (w.tick < 240) w.advance(frames[k++ % frames.size].coerceAtMost((240 - w.tick) / 60.0)) }
            return w.stateHash()
        }
        val stepped = play(null)
        assertEquals(stepped, play(doubleArrayOf(1.0 / 60.0)))
        assertEquals(stepped, play(doubleArrayOf(0.008, 0.016, 0.033, 0.07, 0.0167)))
    }

    @Test fun stopAdvanceEndsTheFrameAfterThisTick() {
        val w = world()
        w.tickListener = object : TickListener {
            override fun afterTick(world: World) { if (world.tick == 2L) world.stopAdvance() }
        }
        val a = w.advance(4.0 / 60.0)
        assertEquals(2L, w.tick, "stopped after the second tick")
        assertEquals(0.0, a, "the rest of the frame is dropped")
        w.advance(1.0 / 60.0)
        assertEquals(3L, w.tick, "the next frame runs again")
        w.stopAdvance() // outside an advance: no effect on the next one
        w.tickListener = null
        w.advance(2.0 / 60.0)
        assertEquals(5L, w.tick)
    }

    @Test fun aListenerCannotStepTheWorldItIsIn() {
        val w = world()
        w.tickListener = object : TickListener {
            override fun beforeTick(world: World) { world.step() }
        }
        assertFailsWith<IllegalStateException> { w.step() }
    }

    // ------------------------------------------------------------ pooled joints

    @Test fun pooledJointsKeepIdsAndRejectStaleOnes() {
        val w = world()
        val a = ghost(w, 30.0); val b = ghost(w, 31.0)
        val first = w.createGrapple(a, b, GrappleDef(breakForce = 1000.0))
        w.destroyJoint(first)
        val second = w.createBlock(a, b, BlockDef(500.0, 500.0, 300.0))
        assertNotEquals(first, second, "same slot, new generation")
        assertFalse(w.hasJoint(first))
        assertTrue(w.hasJoint(second))
        assertEquals(JointType.Block, w.joint(second).type)
        assertFailsWith<IllegalArgumentException> { w.joint(first) }
    }

    @Test fun aReusedJointStartsFresh() {
        // A joint made in a slot that held a strained block behaves exactly like one made in a fresh world.
        fun run(reuse: Boolean): Long {
            val w = world()
            val a = ghost(w, 30.0); val b = ghost(w, 30.8)
            w.setAthlete(b, runner); w.setIntent(b, LocomotionIntent.velocity(0.0, 5.0))
            if (reuse) {
                val old = w.createBlock(a, b, BlockDef(1500.0, 1500.0, 1e9))
                repeat(10) { w.step() }
                w.destroyJoint(old)
                w.restore(World(WorldConfig(), f, 1L).let { fresh ->
                    val fa = ghost(fresh, 30.0); val fb = ghost(fresh, 30.8)
                    fresh.setAthlete(fb, runner); fresh.setIntent(fb, LocomotionIntent.velocity(0.0, 5.0))
                    // Same bodies, same slot generations as the reusing world.
                    fresh.destroyJoint(fresh.createBlock(fa, fb, BlockDef(1.0, 1.0, 1.0)))
                    fresh.snapshot()
                })
            } else {
                w.destroyJoint(w.createBlock(a, b, BlockDef(1.0, 1.0, 1.0)))
            }
            w.createGrapple(a, b, GrappleDef(breakForce = 2000.0, dragForce = 300.0))
            repeat(60) { w.step() }
            return w.stateHash()
        }
        assertEquals(run(false), run(true))
    }

    @Test fun jointPoolFullStillThrows() {
        val w = world(WorldConfig(maxJoints = 2))
        val a = ghost(w, 30.0); val b = ghost(w, 31.0); val c = ghost(w, 32.0)
        w.createGrapple(a, b, GrappleDef(breakForce = 1000.0))
        w.createGrapple(a, c, GrappleDef(breakForce = 1000.0))
        assertFailsWith<IllegalStateException> { w.createGrapple(b, c, GrappleDef(breakForce = 1000.0)) }
    }

    // ------------------------------------------------------------ allocations

    private val mx = ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean
    @Suppress("DEPRECATION") private val tid = Thread.currentThread().id

    private fun supported(): Boolean = mx != null && mx.isThreadAllocatedMemorySupported.also { if (it) mx.isThreadAllocatedMemoryEnabled = true }

    /** Bytes [block] allocates; the JIT may allocate once in a window, so the best of 3 attempts after warm-up. */
    private fun allocated(warm: Int = 2000, block: () -> Unit): Long {
        repeat(warm) { block() }
        var best = Long.MAX_VALUE
        for (attempt in 0 until 3) {
            val before = mx!!.getThreadAllocatedBytes(tid)
            block()
            val d = mx.getThreadAllocatedBytes(tid) - before
            if (d < best) best = d
            if (d == 0L) break
        }
        return best
    }

    @Test fun makingAndBreakingJointsAllocatesNothing() {
        if (!supported()) return
        val w = world()
        val a = ghost(w, 30.0); val b = ghost(w, 31.0); val c = ghost(w, 30.0, y = 10.0)
        w.setAthlete(c, runner)
        val grip = GrappleDef(breakForce = 2000.0, dragForce = 300.0)
        val block = BlockDef(1000.0, 1000.0, 200.0)
        val weld = WeldDef()
        val dist = DistanceDef(hertz = 4.0)
        val go = LocomotionIntent.velocity(9.0, 0.0)
        val weak = GrappleDef(breakForce = 1.0)
        val bytes = allocated {
            repeat(20) {
                val g = w.createGrapple(a, b, grip)
                val k = w.createBlock(b, a, block)
                w.destroyJoint(w.createWeld(a, b, weld))
                w.destroyJoint(w.createDistance(a, b, dist))
                w.destroyJoint(g); w.destroyJoint(k)
            }
            // One that breaks inside a step.
            w.setIntent(c, go)
            val g = w.createGrapple(a, c, weak)
            w.step()
            if (w.hasJoint(g)) w.destroyJoint(g)
        }
        assertEquals(0L, bytes, "bytes allocated making, destroying and breaking joints")
        assertEquals(0, w.jointCount)
    }

    @Test fun theFirstFallAllocatesNothing() {
        if (!supported()) return
        val w = world(WorldConfig(maxBodies = 64))
        val warm = ghost(w, 40.0)
        repeat(2000) { w.setStance(warm, Stance.Down); w.setStance(warm, Stance.Diving); w.setStance(warm, Stance.Upright) }
        val fresh = Array(40) { ghost(w, 20.0 + it, y = 10.0) } // created outside the window
        var k = 0
        val bytes = allocated(warm = 0) {
            // Each body goes down for the first time in its life.
            val id = fresh[k++]
            w.setStance(id, if (k % 2 == 0) Stance.Down else Stance.Diving)
        }
        assertEquals(0L, bytes, "bytes allocated putting a body down for the first time")
        assertEquals(Stance.Diving, w.body(fresh[0]).stance)
    }

    @Test fun theBallCarrierWithoutBoxing() {
        val w = world()
        val a = ghost(w, 30.0); val b = ghost(w, 31.0)
        assertFalse(w.hasBallCarrier)
        assertFailsWith<IllegalStateException> { w.ballCarrierId }
        w.setBallCarrier(a)
        assertTrue(w.hasBallCarrier)
        assertEquals(a, w.ballCarrierId)
        assertEquals(a, w.ballCarrier)
        w.clearBallCarrier()
        assertFalse(w.hasBallCarrier)
        assertEquals(null, w.ballCarrier)
        assertEquals(BallState.Dead, w.ball().state)
        if (!supported()) return
        val bytes = allocated {
            w.setBallCarrier(if (w.tick % 2 == 0L && w.hasBallCarrier && w.ballCarrierId == a) b else a)
            w.step()
        }
        assertEquals(0L, bytes, "bytes allocated reading and setting the ball carrier")
    }
}
