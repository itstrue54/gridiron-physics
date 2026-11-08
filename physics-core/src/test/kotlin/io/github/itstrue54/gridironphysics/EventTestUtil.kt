package io.github.itstrue54.gridironphysics

/** A copy of one event, so tests can collect them (the reader's fields change with every next()). */
data class Ev(
    val type: EventType,
    val tick: Long,
    val a: Int,
    val b: Int,
    val wall: Int,
    val line: Int,
    val direction: Int,
    val sensor: Boolean,
    val px: Double,
    val py: Double,
    val nx: Double,
    val ny: Double,
    val closingSpeed: Double,
    val normalImpulse: Double,
    val angleA: Double,
    val angleB: Double,
    val momAX: Double,
    val momAY: Double,
    val momBX: Double,
    val momBY: Double,
    val value: Double,
)

private val PAIR_TYPES = setOf(EventType.ContactBegin, EventType.ContactEnd, EventType.Hit, EventType.Trip)

/** Reads every pending event. */
fun World.drain(): List<Ev> {
    val r = events()
    val out = ArrayList<Ev>()
    while (r.next()) {
        val b = if (r.type in PAIR_TYPES && r.wall == ContactView.NO_WALL) r.bodyB.value else -1
        out += Ev(
            r.type, r.tick, r.bodyA.value, b, r.wall, r.line, r.direction, r.sensor,
            r.pointX, r.pointY, r.normalX, r.normalY, r.closingSpeed, r.normalImpulse,
            r.angleToHeadingA, r.angleToHeadingB, r.momentumAX, r.momentumAY, r.momentumBX, r.momentumBY,
            r.slipExcess,
        )
    }
    return out
}

/** Steps [ticks] times and returns every event they produced. */
fun World.run(ticks: Int): List<Ev> {
    val all = ArrayList<Ev>()
    repeat(ticks) { step(); all += drain() }
    return all
}
