package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.AthleteParams
import io.github.itstrue54.gridironphysics.BallState
import io.github.itstrue54.gridironphysics.BlockDef
import io.github.itstrue54.gridironphysics.BodyDef
import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.Category
import io.github.itstrue54.gridironphysics.DistanceDef
import io.github.itstrue54.gridironphysics.EventType
import io.github.itstrue54.gridironphysics.FieldConfig
import io.github.itstrue54.gridironphysics.FlightMode
import io.github.itstrue54.gridironphysics.GrappleDef
import io.github.itstrue54.gridironphysics.JointId
import io.github.itstrue54.gridironphysics.LocomotionIntent
import io.github.itstrue54.gridironphysics.Material
import io.github.itstrue54.gridironphysics.Replay
import io.github.itstrue54.gridironphysics.ReplayRecorder
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.Stance
import io.github.itstrue54.gridironphysics.SurfaceConfig
import io.github.itstrue54.gridironphysics.WeldDef
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.WorldConfig
import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.SplitMix64
import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3

/**
 * The determinism harness: [COUNT] seeded random plays. Each one is a small game layer driving a world: random
 * players and athletes, intents and stances, tackles (grapples, then the carrier goes Down), blocks, passes with catch
 * windows and catches, fumbles that bounce loose and get picked up, punts and field goals, deflections, wind and
 * surface changes, bodies created and destroyed mid-play. Its decisions come from its own SplitMix64 and from the
 * world's events and state, so a play is a pure function of its index.
 *
 * Recording starts before setup, after setup, or mid-play (by index), so replays begin from empty and from busy
 * snapshots alike.
 */
object RandomPlays {
    const val COUNT = 1000

    private const val MASTER_SEED = 0x6A1D2026L

    /** The world seed of play [index]. */
    fun seedOf(index: Int): Long = SplitMix64(MASTER_SEED + 0x9E3779B9L * index).nextLong()

    class Result(
        /** The state hash after every tick of the play. */
        val hashes: LongArray,
        /** The recorded replay, when asked for. */
        val replay: Replay?,
        /** Counts of what happened, for checking that the harness exercises the engine. */
        val stats: IntArray,
    )

    // Indexes into Result.stats.
    const val S_GRAPPLES = 0
    const val S_BLOCKS = 1
    const val S_PASSES = 2
    const val S_CATCHES = 3
    const val S_KICKS = 4
    const val S_BOUNCES = 5
    const val S_PICKUPS = 6
    const val S_DOWNS = 7
    const val S_HITS = 8
    const val S_KICK_RESULTS = 9
    const val S_BREAKS = 10
    const val S_CREATED = 11
    const val S_DESTROYED = 12
    const val S_FUMBLES = 13
    const val S_DEFLECTS = 14
    const val S_STATS = 15

    /** Plays play [index] from scratch. With [record], also returns its replay. */
    fun play(index: Int, record: Boolean = false): Result = Play(index, record).run()

    /** FNV-1a over a play's hashes: one number per play for cross-device comparison. */
    fun digest(hashes: LongArray): Long {
        var h = -0x340d631b7bdddcdbL
        for (v in hashes) for (k in 0 until 8) h = (h xor ((v ushr (8 * k)) and 0xFF)) * 0x100000001b3L
        return h
    }

    private class Play(val index: Int, val record: Boolean) {
        val seed = seedOf(index)
        val rng = SplitMix64(seed xor 0x5DEECE66DL)
        val field = if (rng.nextDouble() < 0.2) FieldConfig.COLLEGE else FieldConfig.PRO
        val w = World(WorldConfig(), field, seed)
        val stats = IntArray(S_STATS)
        val ticks = 240 + (rng.nextDouble() * 241).toInt()
        val kind = (rng.nextDouble() * 10).toInt() // 0-3 run, 4-7 pass, 8 punt, 9 field goal
        val recordFrom = when (index % 3) { 0 -> -1; 1 -> 0; else -> 30 + (rng.nextDouble() * 90).toInt() }
        val recorder = ReplayRecorder()

        val offense = ArrayList<BodyId>()
        val defense = ArrayList<BodyId>()
        val nextDecision = IntIntMap()
        val joints = ArrayList<JointId>()
        val grappleOn = ArrayList<BodyId>() // tackler of each live grapple, parallel to grappleIds
        val grappleIds = ArrayList<JointId>()

        val intent = LocomotionIntent()
        val def = BodyDef()
        val v2 = Vec2()
        val from = Vec3()
        val vel = Vec3()
        val tgt = Vec3()
        var dir = if (rng.nextDouble() < 0.5) 1 else -1
        var los = 0.0
        var whistle = -1
        var carrierGrappledTicks = 0

        fun r(lo: Double, hi: Double) = rng.nextDouble(lo, hi)
        fun chance(p: Double) = rng.nextDouble() < p
        fun pick(list: List<BodyId>): BodyId = list[(rng.nextDouble() * list.size).toInt()]

        fun run(): Result {
            if (recordFrom < 0) recorder.start(w)
            setup()
            if (recordFrom == 0) recorder.start(w)
            val hashes = LongArray(ticks)
            for (t in 0 until ticks) {
                if (recordFrom > 0 && t == recordFrom) recorder.start(w)
                commands(t)
                w.step()
                hashes[t] = w.stateHash()
                readEvents()
            }
            recorder.stop()
            return Result(hashes, if (record) recorder.toReplay() else null, stats)
        }

        // ------------------------------------------------------------------ setup

        fun athleteParams(): AthleteParams {
            val rest = r(6.0, 14.0)
            return AthleteParams(
                maxSpeed = r(6.5, 9.6), maxAccel = r(4.0, 9.0), maxDecel = r(6.0, 10.0),
                turnRateAtRest = rest, turnRateAtMax = r(1.5, DMath.min(rest, 4.0)),
                traction = r(0.7, 1.1), balance = r(0.0, 1.0),
            )
        }

        fun player(x: Double, y: Double, home: Boolean): BodyId {
            val cat = if (home) Category.HOME else Category.AWAY
            def.type = BodyType.Dynamic; def.x = x; def.y = y
            def.angle = if (home == (dir > 0)) 0.0 else DMath.PI
            def.vx = 0.0; def.vy = 0.0; def.angularVelocity = 0.0
            def.mass = r(80.0, 150.0); def.linearDamping = 0.0; def.angularDamping = 0.0
            def.shape = ShapeDef.Circle(r(0.30, 0.38)); def.userData = if (home) Tags.HOME else Tags.AWAY
            def.bullet = false; def.categoryBits = cat; def.maskBits = Category.ALL; def.material = Material.PLAYER
            val id = w.createBody(def)
            w.setAthlete(id, athleteParams())
            if (chance(0.6)) w.setReach(id, r(0.9, 1.3), r(2.3, 3.0))
            return id
        }

        fun clampY(y: Double) = DMath.clamp(y, 1.5, field.widthM - 1.5)

        fun setup() {
            w.setSurface(SURFACES[(rng.nextDouble() * SURFACES.size).toInt()])
            w.setWind(r(-6.0, 6.0), r(-6.0, 6.0))
            w.setAttackDirection(dir)
            los = r(25.0, field.lengthM - 25.0)
            val mid = field.widthM / 2
            val nOff = 2 + (rng.nextDouble() * 10).toInt()
            val nDef = 2 + (rng.nextDouble() * 10).toInt()
            for (k in 0 until nOff) {
                val back = if (k == 0) r(3.0, 7.0) else r(0.6, 6.0)
                offense += player(los - dir * back, clampY(mid + (k - nOff / 2) * r(1.4, 3.5) + r(-0.5, 0.5)), home = true)
            }
            for (k in 0 until nDef) {
                defense += player(los + dir * r(0.7, 12.0), clampY(mid + (k - nDef / 2) * r(1.4, 4.0) + r(-0.5, 0.5)), home = false)
            }
            if (chance(0.3)) w.createBody(BodyDef(type = BodyType.Kinematic, x = r(10.0, field.lengthM - 10.0), y = -1.5, vx = r(-1.5, 1.5), shape = ShapeDef.Circle(0.3), categoryBits = Category.OFFICIAL, userData = Tags.OFFICIAL))
            if (chance(0.2)) w.createBody(BodyDef(type = BodyType.Static, x = r(5.0, field.lengthM - 5.0), y = field.widthM + 2.0, shape = ShapeDef.Polygon(doubleArrayOf(-1.0, -0.5, 1.0, -0.5, 0.0, 0.8)), categoryBits = Category.STATIC, material = Material.STRUCTURE, userData = Tags.PROP))
            if (chance(0.15)) w.createBody(BodyDef(x = los + dir * r(15.0, 20.0), y = clampY(mid + r(-15.0, 15.0)), mass = 250.0, linearDamping = 2.0, angularDamping = 2.0, shape = ShapeDef.box(0.6, 1.2), material = Material.STRUCTURE, userData = Tags.PROP))
            // The snap: the first offensive player has the ball.
            w.setBallCarrier(offense[0])
            // Linemen engage the nearest defender within 3 m.
            for (k in 1 until offense.size) {
                val o = offense[k]
                val d = nearest(o, defense, 3.0) ?: continue
                if (joints.size >= MAX_JOINTS) break
                joints += w.createBlock(o, d, BlockDef(blockerPushForce = r(300.0, 1200.0), defenderPushForce = r(300.0, 1200.0), shedThreshold = r(1500.0, 6000.0), driveSpeed = r(1.5, 3.5)))
                stats[S_BLOCKS]++
            }
            for (id in offense) setDecision(id, 0)
            for (id in defense) setDecision(id, 0)
        }

        fun nearest(from: BodyId, among: List<BodyId>, within: Double): BodyId? {
            val f = w.body(from); val fx = f.x; val fy = f.y
            var best: BodyId? = null; var bd = within * within
            for (id in among) {
                val b = w.body(id)
                val d = (b.x - fx) * (b.x - fx) + (b.y - fy) * (b.y - fy)
                if (d < bd && d > 1e-6) { bd = d; best = id }
            }
            return best
        }

        fun dist2(a: BodyId, x: Double, y: Double): Double { val b = w.body(a); return (b.x - x) * (b.x - x) + (b.y - y) * (b.y - y) }

        fun setDecision(id: BodyId, t: Int) = nextDecision.put(id.value, t)

        // ------------------------------------------------------------------ the game's calls each tick

        fun commands(t: Int) {
            val ball = w.ball()
            val carrier = w.ballCarrier
            // Players re-think every few ticks.
            for (id in offense) if (t >= nextDecision.get(id.value)) { decideOffense(id, t, carrier); setDecision(id, t + 4 + (rng.nextDouble() * 26).toInt()) }
            for (id in defense) if (t >= nextDecision.get(id.value)) { decideDefense(id, t, carrier); setDecision(id, t + 3 + (rng.nextDouble() * 20).toInt()) }

            if (whistle < 0) {
                when {
                    kind in 4..7 && t == 40 + index % 50 && carrier == offense[0] -> pass()
                    kind == 8 && t == 25 && carrier == offense[0] -> punt()
                    kind == 9 && t == 25 && carrier == offense[0] -> fieldGoal()
                }
                if (carrier != null) tackles(carrier, t)
                // A loose ball: the nearest player within reach may scoop it up.
                if (ball.state == BallState.Loose) {
                    val all = if (chance(0.5)) offense else defense
                    for (id in all) {
                        if (dist2(id, ball.x, ball.y) < 0.8 * 0.8 && w.body(id).stance == Stance.Upright && chance(0.3)) {
                            w.setBallCarrier(id); stats[S_PICKUPS]++; break
                        }
                    }
                    if (chance(0.01)) { w.deflect(w.ballId, v2.set(r(-0.5, 0.5), r(-0.5, 0.5)), if (chance(0.5)) r(0.2, 1.5) else 0.0); stats[S_DEFLECTS]++ }
                }
                if (ball.state == BallState.Airborne && chance(0.01)) { w.deflect(w.ballId, v2.set(r(-0.3, 0.3), r(-0.3, 0.3)), r(0.0, 0.3)); stats[S_DEFLECTS]++ }
            } else if (t == whistle + 40) {
                // Play over: everyone stops, the ball is dead.
                if (w.ballCarrier != null) w.setBallCarrier(null)
                for (id in offense) w.setIntent(id, intent.setIdle())
                for (id in defense) w.setIntent(id, intent.setIdle())
            }

            // Odd calls that keep every command in the log.
            if (chance(0.004)) w.setWind(r(-8.0, 8.0), r(-8.0, 8.0))
            if (chance(0.0015)) w.setSurface(SURFACES[(rng.nextDouble() * SURFACES.size).toInt()])
            if (chance(0.002)) { dir = -dir; w.setAttackDirection(dir) }
            if (chance(0.004)) { w.deflect(pick(defense), v2.set(r(-60.0, 60.0), r(-60.0, 60.0))); stats[S_DEFLECTS]++ }
            if (chance(0.003)) w.setReach(pick(offense), r(0.0, 1.3), r(0.0, 3.0))
            if (chance(0.003) && offense.size + defense.size < 26) {
                // A substitute runs on.
                val home = chance(0.5)
                val id = player(r(10.0, field.lengthM - 10.0), if (chance(0.5)) 0.5 else field.widthM - 0.5, home)
                (if (home) offense else defense) += id
                setDecision(id, t)
                stats[S_CREATED]++
            }
            if (chance(0.002) && defense.size > 2) {
                val id = pick(defense.subList(1, defense.size))
                removeGrapplesOf(id)
                w.destroyBody(id) // its joints go with it; if it had the ball, the ball goes dead
                defense.remove(id)
                stats[S_DESTROYED]++
            }
            if (chance(0.002) && offense.size >= 3 && joints.size < MAX_JOINTS) {
                val a = offense[1]; val b = offense[2]
                if (dist2(a, w.body(b).x, w.body(b).y) > 0.01) {
                    joints += if (chance(0.5)) w.createDistance(a, b, DistanceDef(hertz = r(1.0, 4.0))) else w.createWeld(a, b, WeldDef(hertz = r(1.0, 4.0)))
                }
            }
            // Some joints are let go by the game.
            if (joints.isNotEmpty() && chance(0.01)) {
                val j = joints[(rng.nextDouble() * joints.size).toInt()]
                if (w.hasJoint(j)) w.destroyJoint(j)
            }
            joints.removeAll { !w.hasJoint(it) }
            var k = 0
            while (k < grappleIds.size) {
                if (!w.hasJoint(grappleIds[k])) { grappleIds.removeAt(k); grappleOn.removeAt(k) } else k++
            }
        }

        fun removeGrapplesOf(id: BodyId) {
            var k = 0
            while (k < grappleOn.size) if (grappleOn[k] == id) { grappleOn.removeAt(k); grappleIds.removeAt(k) } else k++
        }

        fun decideOffense(id: BodyId, t: Int, carrier: BodyId?) {
            val b = w.body(id)
            if (b.stance != Stance.Upright) {
                if (chance(0.3)) w.setStance(id, Stance.Upright)
                return
            }
            if (whistle >= 0) return
            val ball = w.ball()
            when {
                id == carrier -> w.setIntent(id, intent.setVelocity(dir * r(4.0, 9.0), r(-4.0, 4.0), r(0.7, 1.0)))
                ball.state == BallState.Loose && chance(0.6) -> w.setIntent(id, intent.setPoint(ball.x, ball.y, r(0.8, 1.0)))
                chance(0.5) -> w.setIntent(id, intent.setPoint(DMath.clamp(b.x + dir * r(3.0, 20.0), 1.0, field.lengthM - 1.0), clampY(b.y + r(-10.0, 10.0)), r(0.6, 1.0)))
                chance(0.8) -> w.setIntent(id, intent.setVelocity(dir * r(0.0, 8.0), r(-5.0, 5.0), r(0.5, 1.0)))
                else -> w.setIntent(id, intent.setIdle())
            }
        }

        fun decideDefense(id: BodyId, t: Int, carrier: BodyId?) {
            val b = w.body(id)
            val bx = b.x; val by = b.y
            if (b.stance != Stance.Upright) {
                if (chance(0.25)) w.setIntent(id, intent.setIdle().also { it.stance = Stance.Upright })
                intent.stance = null
                return
            }
            if (whistle >= 0) return
            val ball = w.ball()
            when {
                carrier != null && carrier != id && chance(0.85) -> {
                    val c = w.body(carrier)
                    val lead = r(0.0, 0.6)
                    val cx = c.x + c.vx * lead; val cy = c.y + c.vy * lead
                    // Dive at a carrier who is close.
                    if ((cx - bx) * (cx - bx) + (cy - by) * (cy - by) < 2.0 && chance(0.15)) {
                        w.setIntent(id, intent.setPoint(cx, cy).also { it.stance = Stance.Diving })
                        intent.stance = null
                    } else {
                        w.setIntent(id, intent.setPoint(cx, cy, r(0.8, 1.0)))
                    }
                }
                ball.state != BallState.Carried && ball.state != BallState.Dead -> w.setIntent(id, intent.setPoint(ball.x, ball.y, r(0.8, 1.0)))
                else -> w.setIntent(id, intent.setVelocity(r(-6.0, 6.0), r(-6.0, 6.0), r(0.5, 1.0)))
            }
        }

        fun tackles(carrier: BodyId, t: Int) {
            val c = w.body(carrier)
            val cx = c.x; val cy = c.y; val cs = DMath.hypot2(c.vx, c.vy); val stance = c.stance
            var holding = 0
            for (k in grappleOn.indices) if (w.hasJoint(grappleIds[k]) && w.joint(grappleIds[k]).bodyB == carrier) holding++
            for (id in defense) {
                if (joints.size >= MAX_JOINTS || id == carrier) continue
                if (grappleOn.contains(id)) continue
                val d2 = dist2(id, cx, cy)
                if (d2 < 1.1 * 1.1 && d2 > 1e-4 && w.body(id).stance == Stance.Upright && chance(0.25)) {
                    val j = w.createGrapple(id, carrier, GrappleDef(breakForce = r(700.0, 4000.0), dragForce = r(100.0, 700.0)))
                    joints += j; grappleIds += j; grappleOn += id
                    stats[S_GRAPPLES]++
                    holding++
                }
            }
            carrierGrappledTicks = if (holding > 0) carrierGrappledTicks + 1 else 0
            if (stance == Stance.Upright && holding > 0 && chance(0.008)) {
                // Fumble: the ball pops loose.
                w.launchBall(from.set(cx, cy, 1.0), vel.set(c.vx + r(-3.0, 3.0), c.vy + r(-3.0, 3.0), r(1.0, 5.0)), FlightMode.EndOverEnd)
                stats[S_FUMBLES]++
                return
            }
            // The game rules him down: wrapped up long enough, or stopped while held.
            if (stance == Stance.Upright && (carrierGrappledTicks > 25 + index % 20 || (holding >= 2 && cs < 1.0))) {
                w.setStance(carrier, Stance.Down)
                whistle = t
            }
        }

        fun pass() {
            val receivers = offense.subList(1, offense.size)
            if (receivers.isEmpty()) return
            val rcv = pick(receivers)
            val r0 = w.body(rcv)
            val lead = r(0.8, 2.2)
            val tx = DMath.clamp(r0.x + r0.vx * lead + dir * r(0.0, 6.0), 2.0, field.lengthM - 2.0)
            val ty = clampY(r0.y + r0.vy * lead)
            val q = w.body(offense[0])
            from.set(q.x, q.y, 1.9)
            tgt.set(tx, ty, 1.3)
            val mode = if (chance(0.8)) FlightMode.Spiral else FlightMode.Wobble
            if (!(chance(0.5) && w.launchVelocityFor(from, tgt, r(14.0, 26.0), chance(0.2), mode, vel))) {
                w.launchVelocityFor(from, tgt, r(3.5, 14.0), mode, vel)
            }
            w.launchBall(from, vel, mode)
            w.setIntent(rcv, intent.setPoint(tx, ty))
            stats[S_PASSES]++
        }

        fun punt() {
            val q = w.body(offense[0])
            from.set(q.x, q.y, 1.0)
            tgt.set(DMath.clamp(q.x + dir * r(30.0, 50.0), 1.0, field.lengthM - 1.0), clampY(q.y + r(-12.0, 12.0)), 0.0)
            w.launchVelocityFor(from, tgt, r(14.0, 22.0), FlightMode.EndOverEnd, vel)
            w.launchBall(from, vel, FlightMode.EndOverEnd)
            stats[S_KICKS]++
        }

        fun fieldGoal() {
            val q = w.body(offense[0])
            val endX = if (dir > 0) field.lengthM else 0.0
            from.set(q.x, q.y, 0.0)
            tgt.set(endX, field.widthM / 2 + r(-4.5, 4.5), r(2.0, 8.0))
            if (!w.launchVelocityFor(from, tgt, r(24.0, 30.0), false, FlightMode.EndOverEnd, vel)) {
                w.launchVelocityFor(from, tgt, r(10.0, 20.0), FlightMode.EndOverEnd, vel)
            }
            w.launchBall(from, vel, FlightMode.EndOverEnd)
            stats[S_KICKS]++
        }

        fun readEvents() {
            val e = w.events()
            while (e.next()) {
                when (e.type) {
                    EventType.CatchOpportunity -> {
                        // The game's catch rule: a coin flip weighted by side, never after the whistle.
                        val id = e.bodyA
                        if (whistle < 0 && w.ball().state == BallState.Airborne && chance(if (offense.contains(id)) 0.6 else 0.3)) {
                            w.setBallCarrier(id); stats[S_CATCHES]++
                        } else if (chance(0.2) && w.ball().state == BallState.Airborne) {
                            w.deflect(w.ballId, v2.set(r(-1.0, 1.0), r(-1.0, 1.0)), r(0.0, 1.0)); stats[S_DEFLECTS]++
                        }
                    }
                    EventType.BallBounce -> stats[S_BOUNCES]++
                    EventType.Down -> stats[S_DOWNS]++
                    EventType.Hit -> stats[S_HITS]++
                    EventType.KickResult -> stats[S_KICK_RESULTS]++
                    EventType.GrappleBroken, EventType.BlockShed -> stats[S_BREAKS]++
                    else -> {}
                }
            }
        }
    }

    /** A tiny int → int map over parallel arrays (insertion order), so the harness never iterates a HashMap. */
    private class IntIntMap {
        private var keys = IntArray(32)
        private var vals = IntArray(32)
        private var n = 0
        fun get(k: Int): Int {
            for (i in 0 until n) if (keys[i] == k) return vals[i]
            return 0
        }
        fun put(k: Int, v: Int) {
            for (i in 0 until n) if (keys[i] == k) { vals[i] = v; return }
            if (n == keys.size) { keys = keys.copyOf(2 * n); vals = vals.copyOf(2 * n) }
            keys[n] = k; vals[n] = v; n++
        }
    }

    private const val MAX_JOINTS = 40
    private val SURFACES = arrayOf(SurfaceConfig.ARTIFICIAL_TURF, SurfaceConfig.NATURAL_GRASS, SurfaceConfig.WET, SurfaceConfig.FROZEN)
}
