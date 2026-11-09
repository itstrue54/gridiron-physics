package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.ByteSink
import io.github.itstrue54.gridironphysics.internal.ByteSource
import io.github.itstrue54.gridironphysics.internal.Codec
import io.github.itstrue54.gridironphysics.internal.FormatException
import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3

/**
 * A recorded play: the world's settings and seed, a snapshot of where recording began, every command the game made
 * (tick-stamped) and the [World.stateHash] after every tick. [ReplayPlayer] re-runs it and checks each hash.
 *
 * The `.gpr` file is [encode]'s output, little-endian:
 * ```
 * "GPRY" magic (4) · version i32 · seed i64
 * config block   (i32 length + WorldConfig and FieldConfig)
 * snapshot block (i32 length + WorldSnapshot bytes, with their own checksum)
 * command block  (i32 length + commands) · command count i32
 * hash count i32 · hashes (i64 each; hash i is the state after tick startTick + i + 1)
 * ```
 * Each command is `tick i64 · op u8 · payload`, payload fields in the order of the World call's parameters; doubles
 * are raw IEEE bits. There is no checksum over commands and hashes on purpose: a damaged replay still loads, and the
 * player reports the tick where it goes wrong. Reading and writing files is the caller's job.
 */
class Replay internal constructor(
    val config: WorldConfig,
    val field: FieldConfig,
    /** The seed the recorded world was built with (its rng state at the start is in [snapshot]). */
    val seed: Long,
    val snapshot: WorldSnapshot,
    internal val commands: ByteArray,
    /** Commands in the log. */
    val commandCount: Int,
    private val hashes: LongArray,
) {
    /** The tick recording began at. */
    val startTick: Long get() = snapshot.tick

    /** Ticks recorded. */
    val tickCount: Int get() = hashes.size

    /** The tick the recording ends at. */
    val endTick: Long get() = startTick + tickCount

    /** The state hash after tick `startTick + i + 1`. */
    fun hashAt(i: Int): Long = hashes[i]

    /** The `.gpr` bytes. */
    fun encode(): ByteArray {
        val s = ByteSink(64 + commands.size + snapshot.size + 8 * hashes.size + 2048)
        s.i32(MAGIC); s.i32(VERSION); s.i64(seed)
        val c = ByteSink(2048)
        Codec.config(c, config); Codec.field(c, field)
        s.block(c.buf, 0, c.size)
        s.block(snapshot.bytes)
        s.block(commands)
        s.i32(commandCount)
        s.i32(hashes.size)
        for (h in hashes) s.i64(h)
        return s.toByteArray()
    }

    companion object {
        /** Bumped whenever the layout or a command's payload changes. */
        const val VERSION = 1

        internal const val MAGIC = 0x59525047 // "GPRY" little-endian

        /** Reads `.gpr` bytes. Throws [IllegalArgumentException] for a bad header, a damaged snapshot or truncation. */
        fun decode(bytes: ByteArray): Replay {
            val r = ByteSource(bytes)
            require(bytes.size >= 16 && r.i32() == MAGIC) { "not a .gpr replay (bad magic)" }
            val v = r.i32()
            require(v == VERSION) { ".gpr version $v; this build reads version $VERSION" }
            val seed = r.i64()
            val c = r.sub()
            val config = Codec.config(c)
            val field = Codec.field(c)
            c.expectEnd("replay config")
            val snapshot = WorldSnapshot.fromByteArray(r.block())
            val commands = r.block()
            val commandCount = r.count(commands.size, "command")
            val n = r.i32()
            require(n >= 0 && n.toLong() * 8 == r.remaining.toLong()) { "bad hash count $n for ${r.remaining} bytes" }
            val hashes = LongArray(n) { r.i64() }
            r.expectEnd(".gpr")
            return Replay(config, field, seed, snapshot, commands, commandCount, hashes)
        }

        // Command ops. Payloads follow the World call; ids are BodyId/JointId values.
        internal const val OP_CREATE_BODY = 1
        internal const val OP_DESTROY_BODY = 2
        internal const val OP_SET_ATHLETE = 3
        internal const val OP_SET_INTENT = 4
        internal const val OP_SET_STANCE = 5
        internal const val OP_SET_SURFACE = 6
        internal const val OP_CREATE_GRAPPLE = 7
        internal const val OP_CREATE_BLOCK = 8
        internal const val OP_CREATE_DISTANCE = 9
        internal const val OP_CREATE_WELD = 10
        internal const val OP_DESTROY_JOINT = 11
        internal const val OP_SET_BALL_CARRIER = 12
        internal const val OP_ATTACK_DIRECTION = 13
        internal const val OP_LAUNCH_BALL = 14
        internal const val OP_DEFLECT = 15
        internal const val OP_SET_WIND = 16
        internal const val OP_SET_REACH = 17

        internal const val NO_STANCE = 255
    }
}

/**
 * Records a [World] into a [Replay]: [start] snapshots the world and attaches itself as its [World.recorder]; from then
 * on every command and every tick's hash goes into preallocated buffers, so recording never allocates during play as
 * long as the capacities hold (past them a buffer doubles once). [toReplay] can be called at any time.
 */
class ReplayRecorder(commandCapacityBytes: Int = 1 shl 20, tickCapacity: Int = 60 * 60 * 15) : CommandRecorder {
    private val log = ByteSink(commandCapacityBytes)
    private var commandCount = 0
    private var hashes = LongArray(if (tickCapacity < 16) 16 else tickCapacity)
    private var hashCount = 0
    private var world: World? = null
    private var snapshot: WorldSnapshot? = null
    private var config: WorldConfig? = null
    private var field: FieldConfig? = null
    private var seed = 0L

    /** Begins a new recording of [world] from its current state, replacing any earlier one. */
    fun start(world: World) {
        stop()
        snapshot = world.snapshot()
        config = world.config; field = world.field; seed = world.seed
        log.reset(); commandCount = 0; hashCount = 0
        this.world = world
        world.recorder = this
    }

    /** Detaches from the world (the recording so far is kept). */
    fun stop() {
        val w = world ?: return
        if (w.recorder === this) w.recorder = null
        world = null
    }

    /** Ticks recorded so far. */
    val ticksRecorded: Int get() = hashCount

    /** The recording so far, as an independent [Replay]. */
    fun toReplay(): Replay {
        val s = checkNotNull(snapshot) { "nothing recorded: call start(world) first" }
        return Replay(config!!, field!!, seed, s, log.toByteArray(), commandCount, hashes.copyOf(hashCount))
    }

    private fun head(tick: Long, op: Int) {
        log.i64(tick); log.u8(op)
        commandCount++
    }

    override fun onStep(tick: Long, stateHash: Long) {
        if (hashCount == hashes.size) hashes = hashes.copyOf(2 * hashes.size)
        hashes[hashCount++] = stateHash
    }

    override fun onCreateBody(tick: Long, id: BodyId, def: BodyDef) {
        head(tick, Replay.OP_CREATE_BODY)
        log.i32(id.value); log.u8(def.type.ordinal)
        log.f64(def.x); log.f64(def.y); log.f64(def.angle); log.f64(def.vx); log.f64(def.vy); log.f64(def.angularVelocity)
        log.f64(def.mass); log.f64(def.linearDamping); log.f64(def.angularDamping)
        Codec.shape(log, def.shape)
        log.i32(def.userData); log.bool(def.bullet); log.i32(def.categoryBits); log.i32(def.maskBits); log.i32(def.material)
    }

    override fun onDestroyBody(tick: Long, id: BodyId) {
        head(tick, Replay.OP_DESTROY_BODY); log.i32(id.value)
    }

    override fun onSetAthlete(tick: Long, id: BodyId, params: AthleteParams) {
        head(tick, Replay.OP_SET_ATHLETE); log.i32(id.value); Codec.athlete(log, params)
    }

    override fun onSetIntent(tick: Long, id: BodyId, intent: LocomotionIntent) {
        head(tick, Replay.OP_SET_INTENT); log.i32(id.value)
        log.u8(intent.mode.ordinal); log.f64(intent.targetX); log.f64(intent.targetY); log.f64(intent.effort)
        log.u8(intent.stance?.ordinal ?: Replay.NO_STANCE)
    }

    override fun onSetStance(tick: Long, id: BodyId, stance: Stance) {
        head(tick, Replay.OP_SET_STANCE); log.i32(id.value); log.u8(stance.ordinal)
    }

    override fun onSetSurface(tick: Long, surface: SurfaceConfig) {
        head(tick, Replay.OP_SET_SURFACE); Codec.surface(log, surface)
    }

    private fun length(v: Double?) {
        log.bool(v != null)
        if (v != null) log.f64(v)
    }

    override fun onCreateGrapple(tick: Long, id: JointId, tackler: BodyId, carrier: BodyId, def: GrappleDef) {
        head(tick, Replay.OP_CREATE_GRAPPLE); log.i32(id.value); log.i32(tackler.value); log.i32(carrier.value)
        log.f64(def.breakForce); log.f64(def.dragForce); length(def.length); log.f64(def.hertz); log.f64(def.dampingRatio)
    }

    override fun onCreateBlock(tick: Long, id: JointId, blocker: BodyId, defender: BodyId, def: BlockDef) {
        head(tick, Replay.OP_CREATE_BLOCK); log.i32(id.value); log.i32(blocker.value); log.i32(defender.value)
        log.f64(def.blockerPushForce); log.f64(def.defenderPushForce); log.f64(def.shedThreshold); log.f64(def.driveSpeed)
        length(def.length); log.f64(def.hertz); log.f64(def.dampingRatio)
    }

    override fun onCreateDistance(tick: Long, id: JointId, a: BodyId, b: BodyId, def: DistanceDef) {
        head(tick, Replay.OP_CREATE_DISTANCE); log.i32(id.value); log.i32(a.value); log.i32(b.value)
        length(def.length); log.f64(def.hertz); log.f64(def.dampingRatio)
    }

    override fun onCreateWeld(tick: Long, id: JointId, a: BodyId, b: BodyId, def: WeldDef) {
        head(tick, Replay.OP_CREATE_WELD); log.i32(id.value); log.i32(a.value); log.i32(b.value)
        log.f64(def.hertz); log.f64(def.dampingRatio)
    }

    override fun onDestroyJoint(tick: Long, id: JointId) {
        head(tick, Replay.OP_DESTROY_JOINT); log.i32(id.value)
    }

    override fun onSetBallCarrier(tick: Long, id: BodyId?) {
        head(tick, Replay.OP_SET_BALL_CARRIER); log.i32(id?.value ?: -1)
    }

    override fun onSetAttackDirection(tick: Long, direction: Int) {
        head(tick, Replay.OP_ATTACK_DIRECTION); log.i32(direction)
    }

    override fun onLaunchBall(tick: Long, from: Vec3, velocity: Vec3, mode: FlightMode) {
        head(tick, Replay.OP_LAUNCH_BALL)
        log.f64(from.x); log.f64(from.y); log.f64(from.z); log.f64(velocity.x); log.f64(velocity.y); log.f64(velocity.z)
        log.u8(mode.ordinal)
    }

    override fun onDeflect(tick: Long, id: BodyId, impulseX: Double, impulseY: Double, impulseZ: Double) {
        head(tick, Replay.OP_DEFLECT); log.i32(id.value); log.f64(impulseX); log.f64(impulseY); log.f64(impulseZ)
    }

    override fun onSetWind(tick: Long, x: Double, y: Double) {
        head(tick, Replay.OP_SET_WIND); log.f64(x); log.f64(y)
    }

    override fun onSetReach(tick: Long, id: BodyId, reachRadius: Double, reachHeight: Double) {
        head(tick, Replay.OP_SET_REACH); log.i32(id.value); log.f64(reachRadius); log.f64(reachHeight)
    }
}

/**
 * Plays a [Replay] back on a fresh world: before each tick it makes the recorded commands, then steps and compares the
 * state hash with the recorded one. The first tick whose hash differs (or whose commands cannot be applied, or that
 * creates a different id than recorded) is [divergentTick]; playing goes on after it so a viewer can keep showing the
 * play. Allocates (it decodes defs and shapes); it is a tool, not part of a game's frame.
 */
class ReplayPlayer(val replay: Replay) {
    /** The world being replayed, at [World.tick] = startTick + [ticksPlayed]. */
    val world: World = World(replay.config, replay.field, replay.seed)

    /** Ticks stepped so far. */
    var ticksPlayed: Int = 0
        private set

    /** The first tick that did not match the recording, or [NO_DIVERGENCE]. */
    var divergentTick: Long = NO_DIVERGENCE
        private set

    /** What went wrong at [divergentTick], or null. */
    var divergence: String? = null
        private set

    /** True when every recorded tick has been played (or the start state could not be restored). */
    val finished: Boolean get() = ticksPlayed >= replay.tickCount || broken

    private var broken = false
    private val src = ByteSource(replay.commands)
    private var commandsBroken = false
    private val def = BodyDef()
    private val intent = LocomotionIntent()
    private val v2 = Vec2()
    private val from = Vec3()
    private val vel = Vec3()

    init {
        try {
            world.restore(replay.snapshot)
        } catch (e: RuntimeException) {
            diverge(replay.startTick, "the initial snapshot does not fit the replay's settings: ${e.message}")
            broken = true
        }
    }

    /** Plays one tick. Returns false (doing nothing) once [finished]. */
    fun stepOnce(): Boolean {
        if (finished) return false
        val t = world.tick
        applyCommands(t)
        world.step()
        val h = world.stateHash()
        val want = replay.hashAt(ticksPlayed)
        ticksPlayed++
        if (h != want) diverge(world.tick, "state hash ${hex(h)} at tick ${world.tick}, recorded ${hex(want)}")
        return true
    }

    /** Plays to the end and returns [divergentTick]. */
    fun run(): Long {
        while (stepOnce()) { /* keep going */ }
        return divergentTick
    }

    private fun diverge(tick: Long, why: String) {
        if (divergentTick != NO_DIVERGENCE) return
        divergentTick = tick
        divergence = why
    }

    private fun applyCommands(t: Long) {
        while (!commandsBroken && src.remaining > 0) {
            val start = src.pos
            val at: Long
            try {
                at = src.i64()
            } catch (e: IllegalArgumentException) {
                breakCommands(t, "command log is truncated at byte $start"); return
            }
            if (at > t) { src.pos = start; return }
            if (at < t) { breakCommands(t, "command log is out of order: a tick-$at command after tick $t began"); return }
            try {
                apply(t)
            } catch (e: FormatException) {
                // A damaged op or payload: the rest of the stream cannot be framed.
                breakCommands(t, "command log is damaged at tick $t: ${e.message}"); return
            } catch (e: RuntimeException) {
                // A well-formed command the world rejects (a value out of range, a body that is gone).
                diverge(t + 1, "command at tick $t failed: ${e.message}")
            }
        }
    }

    private fun breakCommands(t: Long, why: String) {
        commandsBroken = true
        diverge(t + 1, why)
    }

    private fun idCheck(t: Long, what: String, recorded: Int, got: Int) {
        if (recorded != got) diverge(t + 1, "$what at tick $t got id $got, recorded $recorded")
    }

    private fun apply(t: Long) {
        val r = src
        val w = world
        val op = r.u8()
        // Decoding errors (bad op, enum or shape kind, truncation) are FormatExceptions: the stream is damaged.
        fun <T> pick(values: List<T>, i: Int, what: String): T = if (i < values.size) values[i] else throw FormatException("bad $what $i")
        fun bodyId() = BodyId(r.i32())
        fun len(): Double? = if (r.bool()) r.f64() else null
        when (op) {
            Replay.OP_CREATE_BODY -> {
                val id = r.i32()
                def.type = pick(BodyType.entries, r.u8(), "body type")
                def.x = r.f64(); def.y = r.f64(); def.angle = r.f64(); def.vx = r.f64(); def.vy = r.f64(); def.angularVelocity = r.f64()
                def.mass = r.f64(); def.linearDamping = r.f64(); def.angularDamping = r.f64()
                def.shape = Codec.shape(r)
                def.userData = r.i32(); def.bullet = r.bool(); def.categoryBits = r.i32(); def.maskBits = r.i32(); def.material = r.i32()
                idCheck(t, "createBody", id, w.createBody(def).value)
            }
            Replay.OP_DESTROY_BODY -> w.destroyBody(bodyId())
            Replay.OP_SET_ATHLETE -> { val id = bodyId(); w.setAthlete(id, Codec.athlete(r)) }
            Replay.OP_SET_INTENT -> {
                val id = bodyId()
                val mode = pick(LocomotionIntent.Mode.entries, r.u8(), "intent mode")
                val x = r.f64(); val y = r.f64(); val effort = r.f64()
                val st = r.u8()
                intent.setRaw(mode, x, y, effort, if (st == Replay.NO_STANCE) null else pick(Stance.entries, st, "stance"))
                w.setIntent(id, intent)
            }
            Replay.OP_SET_STANCE -> { val id = bodyId(); w.setStance(id, pick(Stance.entries, r.u8(), "stance")) }
            Replay.OP_SET_SURFACE -> w.setSurface(Codec.surface(r))
            Replay.OP_CREATE_GRAPPLE -> {
                val id = r.i32(); val a = bodyId(); val b = bodyId()
                val bf = r.f64(); val df = r.f64(); val l = len(); val hz = r.f64(); val z = r.f64()
                idCheck(t, "createGrapple", id, w.createGrapple(a, b, GrappleDef(bf, df, l, hz, z)).value)
            }
            Replay.OP_CREATE_BLOCK -> {
                val id = r.i32(); val a = bodyId(); val b = bodyId()
                val pa = r.f64(); val pb = r.f64(); val shed = r.f64(); val drive = r.f64(); val l = len(); val hz = r.f64(); val z = r.f64()
                idCheck(t, "createBlock", id, w.createBlock(a, b, BlockDef(pa, pb, shed, drive, l, hz, z)).value)
            }
            Replay.OP_CREATE_DISTANCE -> {
                val id = r.i32(); val a = bodyId(); val b = bodyId()
                val l = len(); val hz = r.f64(); val z = r.f64()
                idCheck(t, "createDistance", id, w.createDistance(a, b, DistanceDef(l, hz, z)).value)
            }
            Replay.OP_CREATE_WELD -> {
                val id = r.i32(); val a = bodyId(); val b = bodyId()
                val hz = r.f64(); val z = r.f64()
                idCheck(t, "createWeld", id, w.createWeld(a, b, WeldDef(hz, z)).value)
            }
            Replay.OP_DESTROY_JOINT -> w.destroyJoint(JointId(r.i32()))
            Replay.OP_SET_BALL_CARRIER -> { val v = r.i32(); w.setBallCarrier(if (v < 0) null else BodyId(v)) }
            Replay.OP_ATTACK_DIRECTION -> w.setAttackDirection(r.i32())
            Replay.OP_LAUNCH_BALL -> {
                from.set(r.f64(), r.f64(), r.f64()); vel.set(r.f64(), r.f64(), r.f64())
                w.launchBall(from, vel, pick(FlightMode.entries, r.u8(), "flight mode"))
            }
            Replay.OP_DEFLECT -> { val id = bodyId(); val x = r.f64(); val y = r.f64(); w.deflect(id, v2.set(x, y), r.f64()) }
            Replay.OP_SET_WIND -> { val x = r.f64(); w.setWind(x, r.f64()) }
            Replay.OP_SET_REACH -> { val id = bodyId(); val rr = r.f64(); w.setReach(id, rr, r.f64()) }
            else -> throw FormatException("unknown op $op")
        }
    }

    private fun hex(v: Long): String = "0x" + java.lang.Long.toHexString(v)

    companion object {
        const val NO_DIVERGENCE = -1L
    }
}
