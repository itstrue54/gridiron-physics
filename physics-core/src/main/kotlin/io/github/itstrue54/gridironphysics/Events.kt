package io.github.itstrue54.gridironphysics

/** Kinds of event in [World.events]. Which [EventReader] fields each kind sets is listed on the constant. */
enum class EventType {
    /**
     * An intent asked for more horizontal acceleration than traction allows; the engine clamped it. The game may
     * turn it into a stumble. Sets [EventReader.bodyA] and [EventReader.slipExcess].
     */
    Slip,

    /**
     * Two shapes started touching (overlapping, or pushed apart by the solver). Sets bodyA (lower id), bodyB or
     * [EventReader.wall], [EventReader.sensor], the contact point and the normal (A to B).
     */
    ContactBegin,

    /**
     * Two shapes stopped touching, or one of them was destroyed. Sets bodyA, bodyB or wall, and sensor. Every
     * ContactBegin is followed by exactly one ContactEnd for the same pair. Pairs that fall asleep touching
     * stay touching and report nothing.
     */
    ContactEnd,

    /**
     * A player–player contact began (both dynamic, both in [Category.PLAYERS], neither a sensor). Follows its
     * ContactBegin. Sets every hit field: bodyA (lower id), bodyB, point, normal (A to B), closingSpeed,
     * normalImpulse, angleToHeadingA/B and momentumA/B. Players who bounce apart and drive back in make a new
     * ContactBegin and a new (usually softer) Hit; the game decides whether that is one collision or two.
     */
    Hit,

    /**
     * A moving player started touching a [Stance.Down] body. Follows its ContactBegin. bodyA is the moving player
     * (dynamic, in [Category.PLAYERS], not Down, at least [WorldConfig.tripMinSpeed] before the contact), bodyB
     * the Down body. Sets point, normal (mover to Down body) and [EventReader.speed].
     */
    Trip,

    /** A body's shape started overlapping a field line ([EventReader.line]). Sets bodyA and line. */
    SensorBegin,

    /** A body's shape stopped overlapping a field line. Sets bodyA and line. */
    SensorEnd,

    /**
     * A body that was in bounds touched or crossed a sideline or end line (touching the line is out). Sets bodyA
     * and line (the boundary it touched; the lowest [FieldLine] number when it touched two at once). It fires
     * again only after the body is fully back in bounds.
     */
    OutOfBounds,

    /**
     * A body's center crossed a goal line between the sidelines. Sets bodyA, line ([FieldLine.HOME_GOAL_LINE] or
     * [FieldLine.AWAY_GOAL_LINE]), [EventReader.direction] (+1 toward +x) and the crossing point.
     */
    GoalLineCrossed,

    /**
     * A grapple's pull reached its [GrappleDef.breakForce] and the joint broke (it is gone when the event is read).
     * Sets bodyA (tackler), bodyB (carrier), [EventReader.joint] and [EventReader.force] (the pull, N).
     */
    GrappleBroken,

    /**
     * Holding a block squared up took [BlockDef.shedThreshold]: the block is shed and the joint is gone. Sets bodyA
     * (blocker), bodyB (defender), [EventReader.joint] and [EventReader.force] (the lateral force, N).
     */
    BlockShed,

    /**
     * A body's stance became [Stance.Down] (from setStance or an intent's stance). Stamped with the current
     * [World.tick], since stances change between steps. Sets bodyA and [EventReader.ballCarrier]; for the ball
     * carrier also [EventReader.forwardProgressX], where the game spots the ball.
     */
    Down,
}

/**
 * The six field lines the engine watches. They are sensors, not walls: bodies cross them and the engine reports
 * it. Only non-static bodies whose maskBits include [Category.LINE_SENSOR] are watched.
 */
object FieldLine {
    const val NONE = -1

    /** y = 0, from x = 0 to the field length. */
    const val NEAR_SIDELINE = 0

    /** y = field width. */
    const val FAR_SIDELINE = 1

    /** x = 0, across the field. */
    const val HOME_END_LINE = 2

    /** x = field length. */
    const val AWAY_END_LINE = 3

    /** x = end zone depth. */
    const val HOME_GOAL_LINE = 4

    /** x = field length − end zone depth. */
    const val AWAY_GOAL_LINE = 5

    const val COUNT = 6
}

/**
 * The event stream: a preallocated ring buffer the world writes during [World.step] and the game drains. Events
 * stay until read or [clear]ed, so a frame that ran several ticks ([World.advance]) can read them all at once.
 * When the buffer is full the oldest unread event is overwritten and [lostCount] goes up.
 *
 * Read with `while (events.next()) { when (events.type) { … } }`. The field accessors describe the current event
 * (the one the last [next] moved to) and are valid until the next [World.step]. Fields an event type does not set
 * read as 0, except [wall] ([ContactView.NO_WALL]) and [line] ([FieldLine.NONE]); see [EventType]. Nothing here
 * allocates.
 */
class EventReader internal constructor(capacity: Int) {
    private val cap = capacity
    private val types = IntArray(capacity)
    private val ticks = LongArray(capacity)
    private val ints = IntArray(capacity * INTS)
    private val doubles = DoubleArray(capacity * DOUBLES)
    private var head = 0
    private var size = 0
    private var cur = -1

    /** Unread events. */
    val pending: Int get() = size

    /** Events overwritten before they were read, since the world was created. */
    var lostCount: Long = 0L
        private set

    /** Moves to the next unread event and returns true, or returns false when none are left. */
    fun next(): Boolean {
        if (size == 0) { cur = -1; return false }
        cur = head
        head = if (head + 1 == cap) 0 else head + 1
        size--
        return true
    }

    /** Drops every unread event. */
    fun clear() {
        head = 0; size = 0; cur = -1
    }

    private fun slot(): Int {
        check(cur >= 0) { "no current event: call next() first" }
        return cur
    }

    val type: EventType get() = TYPES[types[slot()]]

    /** [World.tick] right after the step that produced the event. */
    val tick: Long get() = ticks[slot()]

    val bodyA: BodyId get() = BodyId(ints[slot() * INTS + I_A])

    /** The other body. Throws for a contact with a runoff wall; check [wall] first. Throws for line events. */
    val bodyB: BodyId
        get() {
            val s = slot()
            check(ints[s * INTS + I_WALL] == ContactView.NO_WALL) { "bodyB is runoff wall ${ints[s * INTS + I_WALL]}, not a body" }
            val t = types[s]
            check(t == T_CONTACT_BEGIN || t == T_CONTACT_END || t == T_HIT || t == T_TRIP || t == T_GRAPPLE_BROKEN || t == T_BLOCK_SHED) { "${TYPES[t]} has no bodyB" }
            return BodyId(ints[s * INTS + I_B])
        }

    /** [ContactView.NO_WALL], or which runoff wall the contact is with. */
    val wall: Int get() = ints[slot() * INTS + I_WALL]

    /** [FieldLine] number for line events, else [FieldLine.NONE]. */
    val line: Int get() = ints[slot() * INTS + I_LINE]

    /** GoalLineCrossed: +1 when moving toward +x, −1 toward −x. */
    val direction: Int get() = ints[slot() * INTS + I_DIR]

    /** Contact events: true when either shape is a sensor or it is a ball–player contact (never pushes). */
    val sensor: Boolean get() = ints[slot() * INTS + I_SENSOR] != 0

    val pointX: Double get() = doubles[slot() * DOUBLES + D_PX]
    val pointY: Double get() = doubles[slot() * DOUBLES + D_PY]
    val normalX: Double get() = doubles[slot() * DOUBLES + D_NX]
    val normalY: Double get() = doubles[slot() * DOUBLES + D_NY]

    /** Hit: relative speed along the normal just before impact (m/s, ≥ 0). */
    val closingSpeed: Double get() = doubles[slot() * DOUBLES + D_CLOSING]

    /** Hit: total normal impulse the solver applied on the first tick (N·s). */
    val normalImpulse: Double get() = doubles[slot() * DOUBLES + D_IMPULSE]

    /** Hit: direction of the contact from A, relative to A's heading (rad, in (−π, π]); 0 = dead ahead, ±π = from behind. */
    val angleToHeadingA: Double get() = doubles[slot() * DOUBLES + D_ANGLE_A]

    /** Hit: direction of the contact from B, relative to B's heading (rad, in (−π, π]). */
    val angleToHeadingB: Double get() = doubles[slot() * DOUBLES + D_ANGLE_B]

    /** Hit: A's momentum just before impact (kg·m/s). */
    val momentumAX: Double get() = doubles[slot() * DOUBLES + D_MAX]
    val momentumAY: Double get() = doubles[slot() * DOUBLES + D_MAY]

    /** Hit: B's momentum just before impact (kg·m/s). */
    val momentumBX: Double get() = doubles[slot() * DOUBLES + D_MBX]
    val momentumBY: Double get() = doubles[slot() * DOUBLES + D_MBY]

    /** Slip: acceleration (m/s²) the intent asked for beyond the traction limit. */
    val slipExcess: Double get() = doubles[slot() * DOUBLES + D_VALUE]

    /** Trip: the moving player's speed just before the contact (m/s). */
    val speed: Double get() = doubles[slot() * DOUBLES + D_VALUE]

    /** GrappleBroken and BlockShed: the joint that broke. Throws for other events. */
    val joint: JointId
        get() {
            val s = slot()
            check(types[s] == T_GRAPPLE_BROKEN || types[s] == T_BLOCK_SHED) { "${TYPES[types[s]]} has no joint" }
            return JointId(ints[s * INTS + I_JOINT])
        }

    /** GrappleBroken: the pull (N); BlockShed: the lateral force (N) when it gave way. */
    val force: Double get() = doubles[slot() * DOUBLES + D_VALUE]

    /**
     * Down: the ball carrier's farthest x toward the attack direction while he was Upright with the ball
     * ([World.forwardProgressX]). 0 when the body is not the carrier.
     */
    val forwardProgressX: Double get() = doubles[slot() * DOUBLES + D_VALUE]

    /** Down: the body is the ball carrier ([World.setBallCarrier]). */
    val ballCarrier: Boolean get() = ints[slot() * INTS + I_CARRIER] != 0

    // ------------------------------------------------------------ writing (World only)

    /** Starts a new event and returns its slot; every field is reset, so callers set only what the type uses. */
    internal fun add(type: Int, tick: Long, a: Int): Int {
        if (size == cap) {
            head = if (head + 1 == cap) 0 else head + 1
            size--
            lostCount++
        }
        var s = head + size
        if (s >= cap) s -= cap
        size++
        types[s] = type
        ticks[s] = tick
        val i = s * INTS
        ints[i + I_A] = a; ints[i + I_B] = 0; ints[i + I_WALL] = ContactView.NO_WALL
        ints[i + I_LINE] = FieldLine.NONE; ints[i + I_DIR] = 0; ints[i + I_SENSOR] = 0
        ints[i + I_JOINT] = -1; ints[i + I_CARRIER] = 0
        val d = s * DOUBLES
        for (k in 0 until DOUBLES) doubles[d + k] = 0.0
        return s
    }

    internal fun setInt(s: Int, field: Int, v: Int) { ints[s * INTS + field] = v }
    internal fun setDouble(s: Int, field: Int, v: Double) { doubles[s * DOUBLES + field] = v }

    internal companion object {
        val TYPES = EventType.values()
        val T_SLIP = EventType.Slip.ordinal
        val T_CONTACT_BEGIN = EventType.ContactBegin.ordinal
        val T_CONTACT_END = EventType.ContactEnd.ordinal
        val T_HIT = EventType.Hit.ordinal
        val T_TRIP = EventType.Trip.ordinal
        val T_SENSOR_BEGIN = EventType.SensorBegin.ordinal
        val T_SENSOR_END = EventType.SensorEnd.ordinal
        val T_OUT_OF_BOUNDS = EventType.OutOfBounds.ordinal
        val T_GOAL_LINE = EventType.GoalLineCrossed.ordinal
        val T_GRAPPLE_BROKEN = EventType.GrappleBroken.ordinal
        val T_BLOCK_SHED = EventType.BlockShed.ordinal
        val T_DOWN = EventType.Down.ordinal

        const val INTS = 8
        const val I_A = 0
        const val I_B = 1
        const val I_WALL = 2
        const val I_LINE = 3
        const val I_DIR = 4
        const val I_SENSOR = 5
        const val I_JOINT = 6
        const val I_CARRIER = 7

        const val DOUBLES = 13
        const val D_PX = 0
        const val D_PY = 1
        const val D_NX = 2
        const val D_NY = 3
        const val D_CLOSING = 4
        const val D_IMPULSE = 5
        const val D_ANGLE_A = 6
        const val D_ANGLE_B = 7
        const val D_MAX = 8
        const val D_MAY = 9
        const val D_MBX = 10
        const val D_MBY = 11
        const val D_VALUE = 12
    }
}
