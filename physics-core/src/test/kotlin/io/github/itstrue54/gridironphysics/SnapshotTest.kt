package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.Vec3
import org.junit.jupiter.api.Assertions.assertArrayEquals
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNotEquals
import kotlin.test.assertTrue

class SnapshotTest {
    /** Every event still in the ring buffer, as text (reading drains it). */
    private fun drainAll(w: World): List<String> {
        val e = w.events()
        val out = ArrayList<String>()
        while (e.next()) {
            val b = when (e.type) {
                EventType.ContactBegin, EventType.ContactEnd, EventType.Hit, EventType.Trip, EventType.GrappleBroken, EventType.BlockShed ->
                    if (e.wall == ContactView.NO_WALL) e.bodyB.value else -100 - e.wall
                else -> 0
            }
            out += "${e.type}@${e.tick} a=${e.bodyA.value} b=$b line=${e.line} p=(${e.pointX.toRawBits()},${e.pointY.toRawBits()},${e.pointZ.toRawBits()}) " +
                "n=(${e.normalX.toRawBits()},${e.normalY.toRawBits()}) v=${e.speed.toRawBits()} i=${e.normalImpulse.toRawBits()} c=${e.closingSpeed.toRawBits()}"
        }
        out += "lost=${e.lostCount}"
        return out
    }

    @Test fun snapshotThenRestoreContinuesIdenticallyFromEveryPhaseOfThePlay() {
        val ticks = ScriptedPlay.TICKS
        val reference = ScriptedPlay().setup()
        val hashes = reference.run(ticks)
        // Snapshot points cover: setup, the pass in the air, the catch, the grapple, the bounce, loose ball, the kick.
        for (at in intArrayOf(0, 15, 31, 45, 120, 225, 262, 290, 345, 361, 432, 460, 540)) {
            val a = ScriptedPlay().setup()
            a.run(at)
            val snap = a.world.snapshot()
            assertEquals(at.toLong(), snap.tick)
            assertEquals(a.world.stateHash(), snap.stateHash)
            // A fresh world, built with a different seed and no bodies, takes the snapshot and plays on.
            val b = ScriptedPlay(seed = 999L).copyScriptFrom(a)
            b.world.restore(snap)
            assertEquals(a.world.stateHash(), b.world.stateHash(), "hash right after restore at $at")
            assertArrayEquals(snap.toByteArray(), b.world.snapshot().toByteArray(), "a restored world snapshots to the same bytes (at $at)")
            val left = ticks - at
            val ha = a.run(left)
            val hb = b.run(left)
            assertArrayEquals(hashes.copyOfRange(at, ticks), ha, "the snapshot did not disturb the original (at $at)")
            assertArrayEquals(ha, hb, "continuation from tick $at")
            assertEquals(drainAll(a.world), drainAll(b.world), "events from tick $at")
            assertArrayEquals(a.world.snapshot().toByteArray(), b.world.snapshot().toByteArray(), "final state from $at")
        }
    }

    @Test fun theScriptedPlayExercisesEverySnapshotFeature() {
        val p = ScriptedPlay().setup()
        val seen = HashSet<String>()
        repeat(ScriptedPlay.TICKS) {
            p.step()
            val w = p.world
            seen += "ball:${w.ball().state}"
            if (w.jointCount > 0) seen += "joints"
            if (w.contactCount > 0) seen += "contacts"
            val e = w.events()
            while (e.next()) seen += "ev:${e.type}"
        }
        for (s in listOf("ball:Airborne", "ball:Loose", "ball:Carried", "ball:Dead", "joints", "contacts", "ev:CatchOpportunity",
            "ev:BallBounce", "ev:KickResult", "ev:Hit", "ev:Down", "ev:ContactBegin", "ev:ContactEnd")) {
            assertTrue(s in seen, "the play must reach $s; saw $seen")
        }
    }

    @Test fun restoreRewindsTheSameWorld() {
        val p = ScriptedPlay().setup()
        p.run(200)
        val snap = p.world.snapshot()
        val tail = p.run(150)
        p.world.restore(snap)
        assertEquals(200L, p.world.tick)
        assertArrayEquals(tail, p.run(150))
    }

    @Test fun snapshotBytesRoundTrip() {
        val p = ScriptedPlay().setup()
        p.run(300)
        val s = p.world.snapshot()
        val copy = WorldSnapshot.fromByteArray(s.toByteArray())
        assertEquals(s.tick, copy.tick)
        assertEquals(s.stateHash, copy.stateHash)
        val w = World(p.config, p.field, 5L)
        w.restore(copy)
        assertEquals(p.world.stateHash(), w.stateHash())
        // The rng state travels: the same bounce yaw afterwards.
        assertEquals(p.world.snapshot().toByteArray().toList(), w.snapshot().toByteArray().toList())
    }

    @Test fun restoreCarriesTheRngState() {
        // Two worlds that differ only in seed bounce differently; after restore they bounce the same.
        fun bouncing(seed: Long): World = World(WorldConfig(), FieldConfig.PRO, seed).also {
            it.launchBall(Vec3(50.0, 24.0, 2.0), Vec3(6.0, 1.0, 9.0), FlightMode.EndOverEnd)
        }
        val a = bouncing(1L); val b = bouncing(2L)
        b.restore(a.snapshot())
        repeat(240) { a.step(); b.step(); assertEquals(a.stateHash(), b.stateHash()) }
        val c = bouncing(2L)
        repeat(240) { c.step() }
        assertNotEquals(a.stateHash(), c.stateHash(), "the seed does matter to bounces")
    }

    @Test fun restoreRejectsAWorldWithAnotherConfigOrField() {
        val s = ScriptedPlay().setup().world.snapshot()
        assertFailsWith<IllegalArgumentException> { World(WorldConfig(maxBodies = 32), FieldConfig.PRO, 1L).restore(s) }
        assertFailsWith<IllegalArgumentException> { World(WorldConfig(velocityIterations = 9), FieldConfig.PRO, 1L).restore(s) }
        assertFailsWith<IllegalArgumentException> { World(WorldConfig(), FieldConfig.COLLEGE, 1L).restore(s) }
        World(WorldConfig(), FieldConfig.PRO, 1L).restore(s) // same settings, other seed: fine
    }

    @Test fun corruptSnapshotBytesAreRejectedBeforeTouchingTheWorld() {
        val p = ScriptedPlay().setup()
        p.run(100)
        val bytes = p.world.snapshot().toByteArray()
        val w = World(p.config, p.field, 1L)
        val before = w.stateHash()
        for (i in intArrayOf(0, 5, 20, bytes.size / 2, bytes.size - 1)) {
            val bad = bytes.copyOf(); bad[i] = (bad[i].toInt() xor 0x10).toByte()
            assertFailsWith<IllegalArgumentException>("flip at $i") { w.restore(WorldSnapshot.fromByteArray(bad)) }
        }
        assertFailsWith<IllegalArgumentException> { WorldSnapshot.fromByteArray(bytes.copyOf(bytes.size - 3)) }
        assertEquals(before, w.stateHash())
    }

    /**
     * The snapshot writes every field of these classes by hand. When a field is added or removed, this fails: write it
     * in World.snapshot / restore (or note here why it is scratch), then update the list.
     */
    @Test fun snapshotCoversEveryStateField() {
        val expected = mapOf(
            "internal.Body" to "angle angle0 angularDamping angularVelocity awake baseMaterial bullet categoryBits extent fx fy geom id inCatch " +
                "inertia invInertia invMass lineMask linearDamping loco lyingShape maskBits mass material outOfBounds preVx preVy prevX prevY " +
                "reachHeight reachRadius shape sleepTime speedBound stance type uprightShape userData vx vy x x0 y y0",
            "internal.Joint" to "a angular angularMass axial axialMass b breakForce broken dragForce dragX dragY driveSpeed id inv k11 k12 k22 " +
                "lateral len linX linY localX localY pushA pushB pushForceA pushForceB rAx rAy refAngle restLength shedThreshold soft solve type ux uy",
            "internal.Contact" to "a b friction idA idB key m maxNormalImpulse normalImpulse normalMass rAx rAy rBx rBy relativeVelocity " +
                "restitution sensor solve tangentImpulse tangentMass wall",
            "internal.Manifold" to "count cpAx cpAy cpBx cpBy id nx ny px py satEdge sep",
            "internal.BallRecord" to "flightMode mode prevX prevY prevZ vx vy vz x y z",
            // stepGain is derived from params.
            "internal.LocomotionController" to "intent params slip stepGain wantsMotion",
            // Only the wind is state; the rest is solver scratch.
            "internal.Flight" to "apex ax ay az c g hitTime hitX hitY outX outY outZ p sMode sPeak sSpeed sTz sUx sUy sX sY sZ windX windY",
            "EventReader" to "cap cur doubles head ints lostCount size ticks types",
            // Written: accumulator alpha attackDirection ball(+ballBody) bodies carrier contactCount contacts eventReader flight(wind)
            // forwardProgressX generations highWater joint* lastTickHash liveCount rng surface tick touch*. The rest is config,
            // views, or per-step scratch that step() rebuilds before reading.
            "World" to "accumulator alpha attackDirection ball ballBody ballId ballSlot ballView bodies broad carrier config contactCount " +
                "contactView contacts eventReader field fingerprint fingerprintSet flight forwardProgressX fp generations highWater " +
                "islandMinTime islandParent jointGenerations jointHighWater jointLive jointView joints lastTickHash liveCount nearDist " +
                "nextA nextB nextCount nextKey nextSensor oldContactCount oldContacts pairA pairB pairCount posts proxyActive proxyCap " +
                "proxyMaxX proxyMaxY proxyMinX proxyMinY ray recorder rigidJoint rng seed softBiasRate softImpulseScale softMassScale " +
                "surface sweepHit tick touchA touchB touchCap touchCount touchKey touchSensor view wallOn walls",
        )
        for ((name, fields) in expected) {
            val c = Class.forName("io.github.itstrue54.gridironphysics.$name")
            val actual = c.declaredFields.filter { !java.lang.reflect.Modifier.isStatic(it.modifiers) && !it.isSynthetic }.map { it.name }.sorted()
            assertEquals(fields.split(" ").sorted(), actual, "fields of $name changed: update the snapshot")
        }
    }

    @Test fun snapshotOfAnEmptyWorldRestores() {
        val a = World(WorldConfig(), FieldConfig.PRO, 3L)
        val b = World(WorldConfig(), FieldConfig.PRO, 4L)
        b.createBody(BodyDef(x = 10.0, y = 10.0))
        b.step()
        b.restore(a.snapshot())
        assertEquals(0, b.bodyCount)
        assertEquals(0L, b.tick)
        assertEquals(a.stateHash(), b.stateHash())
        // Ids continue from the restored generations, not the old world's.
        assertEquals(a.createBody(BodyDef()), b.createBody(BodyDef()))
    }
}
