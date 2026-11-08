package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.BallConfig
import io.github.itstrue54.gridironphysics.FlightMode
import io.github.itstrue54.gridironphysics.WorldConfig
import io.github.itstrue54.gridironphysics.math.DMath

/** A point-mass ball in flight: position and velocity. */
internal class FlightPoint {
    @JvmField var x = 0.0
    @JvmField var y = 0.0
    @JvmField var z = 0.0
    @JvmField var vx = 0.0
    @JvmField var vy = 0.0
    @JvmField var vz = 0.0

    fun set(o: FlightPoint) { x = o.x; y = o.y; z = o.z; vx = o.vx; vy = o.vy; vz = o.vz }
    fun set(x: Double, y: Double, z: Double, vx: Double, vy: Double, vz: Double) {
        this.x = x; this.y = y; this.z = z; this.vx = vx; this.vy = vy; this.vz = vz
    }
}

/**
 * The airborne ball's equations of motion: gravity on z plus quadratic drag against the air (velocity relative to the
 * wind), F = −½ρC_dA|v_rel|v_rel. One fixed-step RK4 tick per [tick]; with no drag the acceleration is constant and RK4
 * is exact, so vacuum flight has no integration error. The world's ball and every prediction (launch solvers,
 * [predict]) run this same code, so a solved launch lands where the solver aimed it bit for bit, posts aside.
 * Allocation-free; not thread-safe.
 */
internal class Flight(private val c: BallConfig, private val g: Double) {
    @JvmField var windX = 0.0
    @JvmField var windY = 0.0

    /** Drag constant k = ½ρC_dA/m for [mode] (acceleration = −k|v_rel|v_rel); 0 with drag off. */
    fun k(mode: Int): Double = if (!c.drag) 0.0 else 0.5 * c.airDensity * c.cd(FlightMode.entries[mode]) * c.crossSection / c.mass

    // Acceleration output of [accel].
    @JvmField var ax = 0.0
    @JvmField var ay = 0.0
    @JvmField var az = 0.0

    fun accel(k: Double, vx: Double, vy: Double, vz: Double) {
        val rx = vx - windX; val ry = vy - windY
        val s = if (k == 0.0) 0.0 else DMath.sqrt(rx * rx + ry * ry + vz * vz)
        ax = -k * s * rx
        ay = -k * s * ry
        az = -g - k * s * vz
    }

    /**
     * Advances [p] by [h] seconds. If the ball comes down through height [zT] on the way (from above it to at or
     * below it) [p] is left at that moment and the time into the step is returned; otherwise returns −1. The crossing
     * is solved on the step's starting state and acceleration (exact in vacuum).
     */
    fun tick(p: FlightPoint, k: Double, h: Double, zT: Double): Double {
        val x0 = p.x; val y0 = p.y; val z0 = p.z
        val vx0 = p.vx; val vy0 = p.vy; val vz0 = p.vz
        accel(k, vx0, vy0, vz0)
        val a1x = ax; val a1y = ay; val a1z = az
        val hh = 0.5 * h
        val v2x = vx0 + hh * a1x; val v2y = vy0 + hh * a1y; val v2z = vz0 + hh * a1z
        accel(k, v2x, v2y, v2z)
        val a2x = ax; val a2y = ay; val a2z = az
        val v3x = vx0 + hh * a2x; val v3y = vy0 + hh * a2y; val v3z = vz0 + hh * a2z
        accel(k, v3x, v3y, v3z)
        val a3x = ax; val a3y = ay; val a3z = az
        val v4x = vx0 + h * a3x; val v4y = vy0 + h * a3y; val v4z = vz0 + h * a3z
        accel(k, v4x, v4y, v4z)
        val s = h / 6.0
        val z1 = z0 + s * (vz0 + 2.0 * v2z + 2.0 * v3z + v4z)
        val above = z0 > zT || (z0 == zT && vz0 <= 0.0)
        if (above && z1 <= zT) {
            // The crossing: zT = z0 + vz0·t + ½a·t², taking the first root in [0, h] (stable form).
            val dz = z0 - zT
            val disc = vz0 * vz0 - 2.0 * a1z * dz
            var t = if (dz <= 0.0) 0.0 else if (disc >= 0.0 && -vz0 + DMath.sqrt(disc) > 0.0) 2.0 * dz / (-vz0 + DMath.sqrt(disc)) else h * dz / (z0 - z1)
            if (t > h) t = h
            p.x = x0 + vx0 * t + 0.5 * a1x * t * t
            p.y = y0 + vy0 * t + 0.5 * a1y * t * t
            p.z = zT
            p.vx = vx0 + a1x * t; p.vy = vy0 + a1y * t; p.vz = vz0 + a1z * t
            return t
        }
        p.x = x0 + s * (vx0 + 2.0 * v2x + 2.0 * v3x + v4x)
        p.y = y0 + s * (vy0 + 2.0 * v2y + 2.0 * v3y + v4y)
        p.z = z1
        p.vx = vx0 + s * (a1x + 2.0 * a2x + 2.0 * a3x + ax)
        p.vy = vy0 + s * (a1y + 2.0 * a2y + 2.0 * a3y + ay)
        p.vz = vz0 + s * (a1z + 2.0 * a2z + 2.0 * a3z + az)
        return -1.0
    }

    // ------------------------------------------------------------ predictions (no posts, no catches, no bounces)

    private val p = FlightPoint()

    /** Output of [predict]: where and when the ball came down through the target height, and its peak. */
    @JvmField var hitX = 0.0
    @JvmField var hitY = 0.0
    @JvmField var hitTime = 0.0
    @JvmField var apex = 0.0

    /**
     * Flies a ball from the given state until it comes down through [zT], ticking exactly as the world does. Returns
     * false when it never does within [MAX_TICKS] (it never rose above zT, or zT > 0 and it reached the ground first).
     * Fills [hitX], [hitY], [hitTime] and [apex] (highest point, solved within the tick).
     */
    fun predict(x: Double, y: Double, z: Double, vx: Double, vy: Double, vz: Double, mode: Int, zT: Double): Boolean {
        val k = k(mode)
        p.set(x, y, z, vx, vy, vz)
        apex = z
        val h = WorldConfig.DT
        for (n in 0 until MAX_TICKS) {
            val z0 = p.z; val vz0 = p.vz
            val t = tick(p, k, h, zT)
            if (vz0 > 0.0 && p.vz <= 0.0) {
                // Peak inside this tick: z0 + vz0²/(2|a|) with the tick's average vertical deceleration.
                val dec = (vz0 - p.vz) / (if (t >= 0.0) t else h)
                if (dec > 0.0) apex = DMath.max(apex, z0 + vz0 * vz0 / (2.0 * dec))
            }
            if (p.z > apex) apex = p.z
            if (t >= 0.0) {
                hitX = p.x; hitY = p.y; hitTime = n * h + t
                return true
            }
            if (zT > 0.0 && p.z <= 0.0) return false
        }
        return false
    }

    /**
     * Seconds until the ball has gone [dist] horizontally along (ux, uy) from its start, or NaN if it lands first.
     * Interpolated linearly within the tick.
     */
    fun timeToDistance(x: Double, y: Double, z: Double, vx: Double, vy: Double, vz: Double, mode: Int, ux: Double, uy: Double, dist: Double): Double {
        if (dist <= 0.0) return 0.0
        val k = k(mode)
        p.set(x, y, z, vx, vy, vz)
        val h = WorldConfig.DT
        var s0 = 0.0
        for (n in 0 until MAX_TICKS) {
            val t = tick(p, k, h, 0.0)
            val s1 = (p.x - x) * ux + (p.y - y) * uy
            if (s1 >= dist) {
                val dt = if (t >= 0.0) t else h
                return n * h + dt * (dist - s0) / (s1 - s0)
            }
            if (t >= 0.0) return Double.NaN
            s0 = s1
        }
        return Double.NaN
    }

    // ------------------------------------------------------------ launch solvers

    /** Solver output velocity. */
    @JvmField var outX = 0.0
    @JvmField var outY = 0.0
    @JvmField var outZ = 0.0

    /**
     * The launch velocity from (x, y, z) that peaks at [apexZ] and comes down through (tx, ty, tz). Starts from the
     * vacuum answer and corrects it against the drag model: the vertical speed by the apex error (Newton step with
     * dApex/dvz ≈ vz/g), the ground velocity by the miss divided by the flight time. Writes [outX]..[outZ].
     */
    fun solveApex(x: Double, y: Double, z: Double, tx: Double, ty: Double, tz: Double, apexZ: Double, mode: Int) {
        var vz = DMath.sqrt(2.0 * g * (apexZ - z))
        val tFlight = vz / g + DMath.sqrt(2.0 * (apexZ - tz) / g)
        var vx = (tx - x) / tFlight
        var vy = (ty - y) / tFlight
        for (it in 0 until SOLVER_ITERATIONS) {
            if (!predict(x, y, z, vx, vy, vz, mode, tz)) {
                // Never came down through tz (cannot happen for apexZ > tz unless the flight is absurdly long): push up.
                vz *= 1.05
                continue
            }
            val ex = tx - hitX; val ey = ty - hitY; val ea = apexZ - apex
            if (DMath.abs(ea) < SOLVER_TOL && ex * ex + ey * ey < SOLVER_TOL * SOLVER_TOL) break
            vz = DMath.max(vz + ea * g / vz, 1e-3)
            vx += ex / hitTime
            vy += ey / hitTime
        }
        outX = vx; outY = vy; outZ = vz
    }

    /**
     * The launch velocity of magnitude [speed] from (x, y, z) that comes down through (tx, ty, tz), on the lower or
     * higher arc. It aims at an aim point A, first the target: for the aim toward A the reach along that direction rises
     * with elevation to a peak and falls again, so the peak is found by golden-section search and the arc's elevation by
     * bisection on its side, giving a reach of |A − from| ([aimAt]). Wind makes the ball come down off target, so A is
     * corrected by Broyden's method (a secant update of the 2×2 response of the landing to A, which can be far from 1
     * on a near-vertical arc in wind), halving any step to an out-of-reach A. Returns true once the miss is under
     * [SPEED_TOL], false when the target is out of reach or the search does not settle.
     */
    fun solveSpeed(x: Double, y: Double, z: Double, tx: Double, ty: Double, tz: Double, speed: Double, high: Boolean, mode: Int): Boolean {
        sSpeed = speed; sMode = mode; sX = x; sY = y; sZ = z; sTz = tz
        if (high) return solveHigh(tx, ty)
        var aX = tx; var aY = ty
        if (!aimAt(aX, aY, high)) return false
        var fX = hitX - tx; var fY = hitY - ty
        // Jacobian estimate of landing w.r.t. A, starting at the identity.
        var j11 = 1.0; var j12 = 0.0; var j21 = 0.0; var j22 = 1.0
        for (it in 0 until AIM_ITERATIONS) {
            if (fX * fX + fY * fY < SPEED_TOL * SPEED_TOL) return true
            val det = j11 * j22 - j12 * j21
            if (!(DMath.abs(det) > 1e-12)) { j11 = 1.0; j12 = 0.0; j21 = 0.0; j22 = 1.0; continue }
            var sX0 = -(j22 * fX - j12 * fY) / det
            var sY0 = -(-j21 * fX + j11 * fY) / det
            var ok = false
            for (half in 0 until 8) {
                if (aimAt(aX + sX0, aY + sY0, high)) { ok = true; break }
                sX0 *= 0.5; sY0 *= 0.5
            }
            if (!ok) return false
            aX += sX0; aY += sY0
            val nX = hitX - tx; val nY = hitY - ty
            // Broyden: J += (ΔF − J·Δa) Δaᵀ / |Δa|².
            val dFX = nX - fX; val dFY = nY - fY
            val ss = sX0 * sX0 + sY0 * sY0
            if (ss > 0.0) {
                val rX = dFX - (j11 * sX0 + j12 * sY0)
                val rY = dFY - (j21 * sX0 + j22 * sY0)
                j11 += rX * sX0 / ss; j12 += rX * sY0 / ss
                j21 += rY * sX0 / ss; j22 += rY * sY0 / ss
            }
            fX = nX; fY = nY
        }
        return fX * fX + fY * fY < SPEED_TOL * SPEED_TOL
    }

    /**
     * The high arc. Its answers can be near vertical, where the landing moves wildly with the aim direction (a lob in
     * a strong wind may have to lean back over the passer's head), so after a first aim at the target ([aimAt]) it is
     * solved in the ground velocity (vx, vy) with vz = +√(speed² − vx² − vy²), which is smooth through vertical: Newton
     * steps on a finite-difference Jacobian, backtracking until the miss shrinks, and kept steeper than the peak-reach
     * elevation so it stays on the high branch.
     */
    private fun solveHigh(tx: Double, ty: Double): Boolean {
        if (!aimAt(tx, ty, true)) return false
        val sp = sSpeed
        val vhMax = sp * DMath.max(DMath.cos(sPeak), 0.0)
        var vx = outX; var vy = outY
        if (!landWith(vx, vy)) return false
        var fX = hitX - tx; var fY = hitY - ty
        val h = 1e-5 * sp
        for (it in 0 until HIGH_ITERATIONS) {
            if (fX * fX + fY * fY < SPEED_TOL * SPEED_TOL) {
                outX = vx; outY = vy; outZ = DMath.sqrt(DMath.max(sp * sp - vx * vx - vy * vy, 0.0))
                return true
            }
            if (!landWith(vx + h, vy)) return false
            val j11 = (hitX - tx - fX) / h; val j21 = (hitY - ty - fY) / h
            if (!landWith(vx, vy + h)) return false
            val j12 = (hitX - tx - fX) / h; val j22 = (hitY - ty - fY) / h
            val det = j11 * j22 - j12 * j21
            if (!(DMath.abs(det) > 1e-18)) return false
            var dx = -(j22 * fX - j12 * fY) / det
            var dy = -(-j21 * fX + j11 * fY) / det
            val len = DMath.hypot2(dx, dy)
            if (len > 0.25 * sp) { dx *= 0.25 * sp / len; dy *= 0.25 * sp / len }
            val f2 = fX * fX + fY * fY
            var accepted = false
            for (half in 0 until 12) {
                var nx = vx + dx; var ny = vy + dy
                val vh = DMath.hypot2(nx, ny)
                if (vh > vhMax) { nx *= vhMax / vh; ny *= vhMax / vh }
                if (landWith(nx, ny)) {
                    val gX = hitX - tx; val gY = hitY - ty
                    if (gX * gX + gY * gY < f2) { vx = nx; vy = ny; fX = gX; fY = gY; accepted = true; break }
                }
                dx *= 0.5; dy *= 0.5
            }
            if (!accepted) return false
        }
        return false
    }

    /** Flies the high-arc launch with ground velocity (vx, vy); false when it never comes down through the target height. */
    private fun landWith(vx: Double, vy: Double): Boolean {
        val vz = DMath.sqrt(DMath.max(sSpeed * sSpeed - vx * vx - vy * vy, 0.0))
        return predict(sX, sY, sZ, vx, vy, vz, sMode, sTz)
    }

    /**
     * Solves the elevation for an aim toward (aX, aY) whose reach along it is the distance there, on the chosen arc.
     * Returns false when the peak reach falls short. On success [outX]..[outZ] hold the velocity and [hitX], [hitY]
     * where it comes down.
     */
    private fun aimAt(aX: Double, aY: Double, high: Boolean): Boolean {
        val dx = aX - sX; val dy = aY - sY
        val dist = DMath.hypot2(dx, dy)
        sUx = if (dist > 0.0) dx / dist else 1.0
        sUy = if (dist > 0.0) dy / dist else 0.0
        var lo = MIN_ELEVATION; var hi = MAX_ELEVATION
        var m1 = hi - GOLDEN * (hi - lo); var m2 = lo + GOLDEN * (hi - lo)
        var f1 = reach(m1); var f2 = reach(m2)
        for (k in 0 until 44) {
            if (f1 < f2) { lo = m1; m1 = m2; f1 = f2; m2 = lo + GOLDEN * (hi - lo); f2 = reach(m2) }
            else { hi = m2; m2 = m1; f2 = f1; m1 = hi - GOLDEN * (hi - lo); f1 = reach(m1) }
        }
        val peak = 0.5 * (lo + hi)
        sPeak = peak
        if (reach(peak) < dist) return false
        // Low arc: reach grows with elevation up to the peak; high arc: it shrinks after it.
        var a = if (high) peak else MIN_ELEVATION
        var b = if (high) MAX_ELEVATION else peak
        for (k in 0 until 52) {
            val mid = 0.5 * (a + b)
            val short = reach(mid) < dist
            if (short == !high) a = mid else b = mid
        }
        val el = 0.5 * (a + b)
        reach(el) // leaves where it comes down in hitX, hitY
        val ce = DMath.cos(el)
        outX = sSpeed * ce * sUx; outY = sSpeed * ce * sUy; outZ = sSpeed * DMath.sin(el)
        return true
    }

    // Scratch for [reach], so the search loops pass no lambdas (no allocation).
    private var sSpeed = 0.0
    private var sPeak = 0.0
    private var sMode = 0
    private var sX = 0.0
    private var sY = 0.0
    private var sZ = 0.0
    private var sTz = 0.0
    private var sUx = 0.0
    private var sUy = 0.0

    /** Distance along the target direction where a launch at elevation [el] comes down through the target height. */
    private fun reach(el: Double): Double {
        val ce = DMath.cos(el)
        val vx = sSpeed * ce * sUx; val vy = sSpeed * ce * sUy; val vz = sSpeed * DMath.sin(el)
        if (!predict(sX, sY, sZ, vx, vy, vz, sMode, sTz)) return -1e9
        return (hitX - sX) * sUx + (hitY - sY) * sUy
    }

    private companion object {
        /** 60 s of flight. */
        const val MAX_TICKS = 3600
        const val SOLVER_ITERATIONS = 200
        const val AIM_ITERATIONS = 30
        const val HIGH_ITERATIONS = 40
        const val SPEED_TOL = 1e-3
        const val SOLVER_TOL = 1e-4
        const val GOLDEN = 0.6180339887498949
        const val MIN_ELEVATION = -1.3962634015954636 // −80°
        /**
         * 170°: past vertical the throw tilts back over the passer's head. A high arc in a strong wind can need that,
         * when the wind carries a near-vertical lob back past him to a target on the other side.
         */
        const val MAX_ELEVATION = 2.9670597283903604
    }
}
