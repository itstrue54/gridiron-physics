package io.github.itstrue54.gridironphysics.internal

/**
 * Little-endian byte writer over a growable array. Writing never allocates until [buf] is full; then it doubles.
 * Doubles are written as their raw bits, so every value (NaN payloads included) round-trips exactly.
 */
internal class ByteSink(capacity: Int = 256) {
    @JvmField var buf = ByteArray(if (capacity < 16) 16 else capacity)
    @JvmField var size = 0

    fun reset() { size = 0 }

    private fun ensure(n: Int) {
        val need = size + n
        if (need <= buf.size) return
        var cap = buf.size * 2
        if (cap < need) cap = need
        buf = buf.copyOf(cap)
    }

    fun u8(v: Int) {
        ensure(1)
        buf[size++] = v.toByte()
    }

    fun bool(v: Boolean) = u8(if (v) 1 else 0)

    fun i32(v: Int) {
        ensure(4)
        val b = buf
        var p = size
        b[p++] = v.toByte(); b[p++] = (v ushr 8).toByte(); b[p++] = (v ushr 16).toByte(); b[p++] = (v ushr 24).toByte()
        size = p
    }

    fun i64(v: Long) {
        ensure(8)
        val b = buf
        var p = size
        for (k in 0 until 8) b[p++] = (v ushr (8 * k)).toByte()
        size = p
    }

    fun f64(v: Double) = i64(java.lang.Double.doubleToRawLongBits(v))

    /** A string as its length and UTF-16 code units (no allocation, unlike an encoder). */
    fun str(s: String) {
        i32(s.length)
        for (i in 0 until s.length) {
            val c = s[i].code
            u8(c); u8(c ushr 8)
        }
    }

    /** A length-prefixed block. */
    fun block(b: ByteArray, off: Int = 0, len: Int = b.size - off) {
        i32(len)
        ensure(len)
        System.arraycopy(b, off, buf, size, len)
        size += len
    }

    fun toByteArray(): ByteArray = buf.copyOf(size)
}

/**
 * Little-endian reader over [buf] from [pos] to [end]. Every read checks bounds and throws [IllegalArgumentException]
 * on truncated or malformed input, so a corrupt file never reads out of range.
 */
internal class ByteSource(private val buf: ByteArray, @JvmField var pos: Int = 0, private val end: Int = buf.size) {
    val remaining: Int get() = end - pos

    private fun need(n: Int) {
        if (n < 0 || n > end - pos) throw FormatException("truncated: need $n bytes at offset $pos, have ${end - pos}")
    }

    fun u8(): Int {
        need(1)
        return buf[pos++].toInt() and 0xFF
    }

    fun bool(): Boolean = when (val v = u8()) {
        0 -> false
        1 -> true
        else -> throw FormatException("bad boolean $v at offset ${pos - 1}")
    }

    fun i32(): Int {
        need(4)
        val b = buf
        val p = pos
        pos = p + 4
        return (b[p].toInt() and 0xFF) or ((b[p + 1].toInt() and 0xFF) shl 8) or
            ((b[p + 2].toInt() and 0xFF) shl 16) or ((b[p + 3].toInt() and 0xFF) shl 24)
    }

    fun i64(): Long {
        need(8)
        var v = 0L
        for (k in 0 until 8) v = v or ((buf[pos + k].toLong() and 0xFF) shl (8 * k))
        pos += 8
        return v
    }

    fun f64(): Double = java.lang.Double.longBitsToDouble(i64())

    /** A count that must be in 0..[max]. */
    fun count(max: Int, what: String): Int {
        val n = i32()
        if (n < 0 || n > max) throw FormatException("bad $what count $n (max $max) at offset ${pos - 4}")
        return n
    }

    /** A length prefix that must fit in the bytes left after it. */
    private fun length(what: String): Int {
        val n = i32()
        if (n < 0 || n > end - pos) throw FormatException("bad $what length $n at offset ${pos - 4}, ${end - pos} bytes left")
        return n
    }

    fun str(): String {
        val n = length("string")
        need(2 * n)
        val c = CharArray(n)
        for (i in 0 until n) c[i] = (u8() or (u8() shl 8)).toChar()
        return String(c)
    }

    /** A length-prefixed block, copied out. */
    fun block(): ByteArray {
        val n = length("block")
        val out = buf.copyOfRange(pos, pos + n)
        pos += n
        return out
    }

    /** A length-prefixed block as a reader over the same array (no copy). */
    fun sub(): ByteSource {
        val n = length("block")
        val s = ByteSource(buf, pos, pos + n)
        pos += n
        return s
    }

    fun expectEnd(what: String) {
        if (pos != end) throw FormatException("$what: ${end - pos} unexpected trailing bytes at offset $pos")
    }
}

/** Malformed or truncated bytes (as opposed to well-formed values a constructor rejects). */
internal class FormatException(message: String) : IllegalArgumentException(message)
