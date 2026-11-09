package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.internal.ByteSource
import io.github.itstrue54.gridironphysics.math.Fnv1a64

/**
 * The complete state of a [World] at one tick, as bytes: every body (athletes, intents, stances, sleep, reach, line
 * state), joints, contacts with their warm-start impulses, the touching set, the ball and the loose-ball body, wind,
 * surface, carrier, forward progress, attack direction, the event ring buffer and the rng. Immutable.
 *
 * [World.restore] needs a world built with the same [WorldConfig] and [FieldConfig]; its seed does not matter. The
 * bytes are little-endian with a version header and an FNV-1a checksum, so they can be stored and moved between
 * devices; [fromByteArray] rejects anything damaged.
 */
class WorldSnapshot internal constructor(internal val bytes: ByteArray) {
    /** The tick the snapshot was taken at ([World.tick]). */
    val tick: Long get() = ByteSource(bytes, TICK_OFFSET).i64()

    /** [World.stateHash] at that tick: a world restored from this snapshot has the same hash. */
    val stateHash: Long get() = ByteSource(bytes, HASH_OFFSET).i64()

    /** Size in bytes. */
    val size: Int get() = bytes.size

    /** A copy of the bytes. */
    fun toByteArray(): ByteArray = bytes.copyOf()

    companion object {
        /** Bumped whenever the layout changes; older snapshots are rejected, not misread. */
        const val VERSION = 1

        internal const val MAGIC = 0x4E535047 // "GPSN" little-endian
        internal const val FINGERPRINT_OFFSET = 8
        internal const val TICK_OFFSET = 16
        internal const val HASH_OFFSET = 24
        internal const val HEADER = 32

        /** Checks the header and checksum of [bytes] (which are copied) and wraps them. Throws [IllegalArgumentException] when damaged. */
        fun fromByteArray(bytes: ByteArray): WorldSnapshot {
            validate(bytes)
            return WorldSnapshot(bytes.copyOf())
        }

        internal fun validate(b: ByteArray) {
            require(b.size >= HEADER + 8) { "not a world snapshot: ${b.size} bytes" }
            val r = ByteSource(b)
            require(r.i32() == MAGIC) { "not a world snapshot (bad magic)" }
            val v = r.i32()
            require(v == VERSION) { "world snapshot version $v; this build reads version $VERSION" }
            val stored = ByteSource(b, b.size - 8).i64()
            require(checksum(b, b.size - 8) == stored) { "world snapshot is corrupt (checksum mismatch)" }
        }

        internal fun checksum(b: ByteArray, len: Int): Long {
            var h = Fnv1a64.OFFSET
            for (i in 0 until len) h = Fnv1a64.mixByte(h, b[i].toInt())
            return h
        }
    }
}
