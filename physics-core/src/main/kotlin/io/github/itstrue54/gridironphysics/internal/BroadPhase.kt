package io.github.itstrue54.gridironphysics.internal

/**
 * Finds proxy pairs whose AABBs overlap. Proxies are indexed so that index order is body-id order; every
 * implementation must write pairs as (lower, higher) sorted by (lower, higher) for determinism. A uniform
 * grid can replace the brute-force version later without touching the solver.
 */
internal interface BroadPhase {
    /** Writes at most `outA.size` pairs among proxies `0 until n` with `active[i]`; returns how many. */
    fun findPairs(
        n: Int, active: BooleanArray,
        minX: DoubleArray, minY: DoubleArray, maxX: DoubleArray, maxY: DoubleArray,
        outA: IntArray, outB: IntArray,
    ): Int
}

/** v1: tests all pairs. About 300 pairs for 25 bodies. */
internal class BruteForceBroadPhase : BroadPhase {
    override fun findPairs(
        n: Int, active: BooleanArray,
        minX: DoubleArray, minY: DoubleArray, maxX: DoubleArray, maxY: DoubleArray,
        outA: IntArray, outB: IntArray,
    ): Int {
        var count = 0
        val cap = outA.size
        for (i in 0 until n) {
            if (!active[i]) continue
            for (j in i + 1 until n) {
                if (!active[j]) continue
                if (minX[i] > maxX[j] || minX[j] > maxX[i] || minY[i] > maxY[j] || minY[j] > maxY[i]) continue
                if (count == cap) return count
                outA[count] = i; outB[count] = j
                count++
            }
        }
        return count
    }
}
