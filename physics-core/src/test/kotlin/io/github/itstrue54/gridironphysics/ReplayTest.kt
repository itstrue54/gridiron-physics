package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3
import org.junit.jupiter.api.Assertions.assertArrayEquals
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class ReplayTest {
    /** The scripted play, recorded from tick [startAt] on. Returns the replay and the hashes the play produced. */
    private fun record(startAt: Int = 0, ticks: Int = ScriptedPlay.TICKS): Pair<Replay, LongArray> {
        val p = ScriptedPlay().setup()
        p.run(startAt)
        val rec = ReplayRecorder()
        rec.start(p.world)
        val hashes = p.run(ticks - startAt)
        rec.stop()
        return rec.toReplay() to hashes
    }

    @Test fun aRecordedPlayReplaysWithEveryHashMatching() {
        for (start in intArrayOf(0, 31, 262)) {
            val (replay, hashes) = record(start)
            assertEquals(start.toLong(), replay.startTick)
            assertEquals(hashes.size, replay.tickCount)
            for (i in hashes.indices) assertEquals(hashes[i], replay.hashAt(i))
            val player = ReplayPlayer(replay)
            assertEquals(ReplayPlayer.NO_DIVERGENCE, player.run(), "from $start: ${player.divergence}")
            assertTrue(player.finished)
            assertEquals(ScriptedPlay.TICKS.toLong(), player.world.tick)
        }
    }

    @Test fun recordingIncludesSetupCommandsWhenStartedOnAnEmptyWorld() {
        val p = ScriptedPlay()
        val rec = ReplayRecorder()
        rec.start(p.world)
        p.setup()
        p.run(200)
        val replay = rec.toReplay()
        assertEquals(0L, replay.startTick)
        assertTrue(replay.commandCount > 30, "setup is in the log: ${replay.commandCount}")
        assertEquals(ReplayPlayer.NO_DIVERGENCE, ReplayPlayer(replay).run())
    }

    @Test fun encodeDecodeRoundTrip() {
        val (replay, _) = record(31)
        val bytes = replay.encode()
        val back = Replay.decode(bytes)
        assertArrayEquals(bytes, back.encode(), "decode then encode gives the same bytes")
        assertEquals(replay.seed, back.seed)
        assertEquals(replay.startTick, back.startTick)
        assertEquals(replay.tickCount, back.tickCount)
        assertEquals(replay.commandCount, back.commandCount)
        assertEquals(replay.config.copy(materials = MaterialTable.DEFAULT), back.config.copy(materials = MaterialTable.DEFAULT))
        assertEquals(replay.field, back.field)
        assertEquals(ReplayPlayer.NO_DIVERGENCE, ReplayPlayer(back).run())
        // Little-endian magic and version up front.
        assertEquals('G'.code.toByte(), bytes[0]); assertEquals('P'.code.toByte(), bytes[1]); assertEquals('R'.code.toByte(), bytes[2])
        assertEquals(Replay.VERSION.toByte(), bytes[4]); assertEquals(0.toByte(), bytes[5])
    }

    @Test fun aCorruptedHashIsFlaggedAtItsTick() {
        val (replay, _) = record(0)
        val bytes = replay.encode()
        for (k in intArrayOf(0, 1, 137, replay.tickCount - 1)) {
            val bad = bytes.copyOf()
            val at = bad.size - 8 * (replay.tickCount - k) // hash k's first byte
            bad[at] = (bad[at].toInt() xor 1).toByte()
            val player = ReplayPlayer(Replay.decode(bad))
            assertEquals(replay.startTick + k + 1, player.run(), "corrupted hash $k")
            assertTrue(player.divergence!!.contains("hash"), player.divergence)
        }
    }

    @Test fun aCorruptedCommandIsFlaggedAtTheTickItChanges() {
        // The script's tick-70 wind (1.0, -0.5) acts on the pass in the air at once: change its x a little in the file
        // and the replay diverges right after tick 70.
        val (replay, _) = record(0)
        val bytes = replay.encode()
        val needle = longBytes(70L) + byteArrayOf(Replay.OP_SET_WIND.toByte()) + longBytes(java.lang.Double.doubleToRawLongBits(1.0))
        val at = indexOf(bytes, needle)
        assertTrue(at > 0, "the wind command is in the file")
        val bad = bytes.copyOf()
        bad[at + 9 + 5] = (bad[at + 9 + 5].toInt() xor 0x01).toByte() // a small change in the mantissa
        val player = ReplayPlayer(Replay.decode(bad))
        assertEquals(71L, player.run(), player.divergence)
        // The player keeps going after a divergence (the viewer shows it), but the first divergent tick sticks.
        assertTrue(player.finished)
        assertEquals(71L, player.divergentTick)
    }

    @Test fun aBrokenCommandStreamIsFlaggedAtItsTick() {
        val (replay, _) = record(0)
        val bytes = replay.encode()
        // The tick-45 attack-direction command: its tick stamp, then op, then -1.
        val needle = longBytes(45L) + byteArrayOf(Replay.OP_ATTACK_DIRECTION.toByte()) + intBytes(-1)
        val at = indexOf(bytes, needle)
        assertTrue(at > 0)
        val bad = bytes.copyOf()
        bad[at + 8] = 0x7F // an unknown op
        val player = ReplayPlayer(Replay.decode(bad))
        assertEquals(46L, player.run())
        assertTrue(player.divergence!!.contains("command"), player.divergence)
    }

    @Test fun damagedHeadersAreRejected() {
        val (replay, _) = record(31, 100)
        val bytes = replay.encode()
        assertFailsWith<IllegalArgumentException> { Replay.decode(bytes.copyOf(10)) }
        assertFailsWith<IllegalArgumentException> { Replay.decode(bytes.copyOf(bytes.size - 1)) }
        assertFailsWith<IllegalArgumentException> { Replay.decode(bytes.copyOf().also { it[0] = 'X'.code.toByte() }) }
        assertFailsWith<IllegalArgumentException> { Replay.decode(bytes.copyOf().also { it[4] = 99 }) }
        // A damaged initial snapshot fails its checksum.
        val snap = indexOf(bytes, intBytes(0x4E535047)) // "GPSN"
        assertTrue(snap > 0)
        assertFailsWith<IllegalArgumentException> { Replay.decode(bytes.copyOf().also { it[snap + 60] = (it[snap + 60].toInt() xor 4).toByte() }) }
    }

    @Test fun damagedLengthsAreFormatErrorsNotOutOfBoundsReads() {
        val (replay, _) = record(31, 100)
        val bytes = replay.encode()
        // The config block's length sits right after magic, version and seed. Every bad length must be rejected cleanly.
        for (len in intArrayOf(-1, bytes.size - 20, bytes.size - 19, bytes.size - 16, Int.MAX_VALUE)) {
            val bad = bytes.copyOf()
            for (k in 0 until 4) bad[16 + k] = (len ushr (8 * k)).toByte()
            assertFailsWith<IllegalArgumentException>("config length $len") { Replay.decode(bad) }
        }
        // Too many hashes for the bytes left.
        val bad = bytes.copyOf()
        val at = bad.size - 8 * replay.tickCount - 4
        bad[at] = (bad[at] + 1).toByte()
        assertFailsWith<IllegalArgumentException> { Replay.decode(bad) }
    }

    @Test fun everyCommandRoundTripsThroughTheLog() {
        // One of each command, with awkward values (NaN payloads aside, every double travels as raw bits).
        val w = World(WorldConfig(), FieldConfig.COLLEGE, 9L)
        val rec = ReplayRecorder()
        rec.start(w)
        val a = w.createBody(BodyDef(x = 30.0, y = 20.0, angle = -0.1, vx = 1.0 / 3.0, mass = 101.5, shape = ShapeDef.Circle(0.35), userData = 7, bullet = true, categoryBits = Category.AWAY, maskBits = 0x00FF, material = Material.PLAYER))
        val b = w.createBody(BodyDef(x = 31.0, y = 20.0, shape = ShapeDef.box(0.3, 0.4), linearDamping = 0.5, angularDamping = 0.25))
        val s = w.createBody(BodyDef(type = BodyType.Static, x = 10.0, y = 10.0, shape = ShapeDef.Segment(0.0, 0.0, 1.0, 1.0)))
        val k = w.createBody(BodyDef(type = BodyType.Kinematic, x = 40.0, y = 1.0, vx = 1.0, shape = ShapeDef.Capsule(0.5, 0.2, sensor = true)))
        w.setAthlete(a, AthleteParams(maxSpeed = 9.1, balance = 0.9))
        w.setIntent(a, LocomotionIntent().setPoint(40.0, 21.0, 0.8).also { it.stance = Stance.Diving })
        w.setStance(a, Stance.Upright)
        w.setSurface(SurfaceConfig("Mud", 0.6, 0.4))
        w.step()
        val g = w.createGrapple(b, a, GrappleDef(breakForce = Double.POSITIVE_INFINITY, dragForce = 50.0, length = 1.0, hertz = 3.0, dampingRatio = 0.5))
        val bl = w.createBlock(a, b, BlockDef(blockerPushForce = 100.0, defenderPushForce = 90.0, shedThreshold = 500.0, driveSpeed = 2.0, hertz = 4.0, dampingRatio = 0.8))
        w.createDistance(a, b, DistanceDef(length = null, hertz = 0.0))
        w.createWeld(a, b, WeldDef(hertz = 2.0, dampingRatio = 0.3))
        w.destroyJoint(g)
        w.destroyJoint(bl)
        w.setBallCarrier(a)
        w.setAttackDirection(-1)
        w.step()
        w.launchBall(Vec3(30.0, 20.0, 1.8), Vec3(8.0, 0.5, 6.0), FlightMode.Wobble)
        w.deflect(w.ballId, Vec2(0.1, -0.2), 0.05)
        w.deflect(b, Vec2(5.0, 0.0))
        w.setWind(-3.5, 0.25)
        w.setReach(a, 1.0, 2.5)
        w.step()
        w.setBallCarrier(null)
        w.destroyBody(s)
        w.destroyBody(k)
        w.step()
        rec.stop()
        val replay = Replay.decode(rec.toReplay().encode())
        assertEquals(24, replay.commandCount)
        val p = ReplayPlayer(replay)
        assertEquals(ReplayPlayer.NO_DIVERGENCE, p.run(), p.divergence)
        assertArrayEquals(w.snapshot().toByteArray(), p.world.snapshot().toByteArray(), "the replayed world ends in exactly the same state")
        assertEquals("Mud", p.world.surface.name)
    }

    @Test fun recordingDoesNotAllocateDuringPlay() {
        val mx = java.lang.management.ManagementFactory.getThreadMXBean() as? com.sun.management.ThreadMXBean ?: return
        if (!mx.isThreadAllocatedMemorySupported) return
        // Everything the game passes is made up front (a nullable id is boxed once, here). Joint creation is left out:
        // World allocates the Joint record itself (M5), recorder or not.
        val intent = LocomotionIntent()
        val imp = Vec2(); val from = Vec3(); val vel = Vec3()
        val ids = IntArray(64)
        fun play(w: World, ticks: Int, carrierArg: BodyId?) {
            val n = w.bodyIds(ids)
            for (t in 0 until ticks) {
                val id = BodyId(ids[t % 7])
                w.setIntent(id, intent.setVelocity((t % 9) - 4.0, (t % 5) - 2.0, 0.9))
                when (t % 60) {
                    5 -> w.setStance(BodyId(ids[3]), Stance.Diving)
                    20 -> w.setStance(BodyId(ids[3]), Stance.Upright)
                    25 -> w.setWind(t * 0.01, -1.0)
                    30 -> w.setReach(id, 1.0, 2.6)
                    35 -> w.setBallCarrier(carrierArg)
                    40 -> w.launchBall(from.set(50.0, 24.0, 1.5), vel.set(6.0, 1.0, 5.0), FlightMode.Spiral)
                    41 -> w.deflect(w.ballId, imp.set(0.1, 0.0), 0.0)
                    45 -> w.deflect(id, imp.set(30.0, 0.0))
                    50 -> w.setAttackDirection(if (t % 120 < 60) -1 else 1)
                    55 -> w.setSurface(SurfaceConfig.WET)
                }
                w.step()
            }
            check(n > 7)
        }
        fun fresh(): Pair<World, BodyId?> {
            val p = ScriptedPlay().setup()
            p.run(330)
            val boxed: BodyId? = BodyId(p.world.bodyIds(ids).let { ids[2] })
            // A body's lying capsule is built on its first dive (once, by World): build it before measuring.
            p.world.setStance(BodyId(ids[3]), Stance.Diving)
            p.world.setStance(BodyId(ids[3]), Stance.Upright)
            return p.world to boxed
        }
        repeat(4) { val (w, c) = fresh(); ReplayRecorder().start(w); play(w, 600, c) }
        val tid = Thread.currentThread().id
        val attempts = ArrayList<Long>()
        while (attempts.size < 3 && (attempts.isEmpty() || attempts.last() != 0L)) {
            val (w, c) = fresh()
            val rec = ReplayRecorder()
            rec.start(w) // the snapshot allocates; recording from here on must not
            val before = mx.getThreadAllocatedBytes(tid)
            play(w, 600, c)
            attempts += mx.getThreadAllocatedBytes(tid) - before
            rec.stop()
            assertEquals(600, rec.toReplay().tickCount)
        }
        assertEquals(0L, attempts.last(), "bytes allocated by 600 recorded steps and their commands, per attempt $attempts")
    }

    private fun longBytes(v: Long) = ByteArray(8) { (v ushr (8 * it)).toByte() }
    private fun intBytes(v: Int) = ByteArray(4) { (v ushr (8 * it)).toByte() }
    private fun indexOf(h: ByteArray, n: ByteArray): Int {
        outer@ for (i in 0..h.size - n.size) {
            for (j in n.indices) if (h[i + j] != n[j]) continue@outer
            return i
        }
        return -1
    }
}
