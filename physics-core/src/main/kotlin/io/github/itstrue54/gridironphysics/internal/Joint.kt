package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.JointType
import io.github.itstrue54.gridironphysics.WorldConfig
import io.github.itstrue54.gridironphysics.math.DMath

/** Catto's soft-step coefficients for one spring frequency and damping ratio at the fixed timestep. */
internal class Softness private constructor(
    @JvmField val biasRate: Double,
    @JvmField val massScale: Double,
    @JvmField val impulseScale: Double,
) {
    companion object {
        fun of(hertz: Double, dampingRatio: Double): Softness {
            if (hertz == 0.0) return RIGID_RELAX
            val h = WorldConfig.DT
            val omega = 2.0 * DMath.PI * hertz
            val a1 = 2.0 * dampingRatio + h * omega
            val a2 = h * omega * a1
            val a3 = 1.0 / (1.0 + a2)
            return Softness(omega / a1, a2 * a3, a3)
        }

        /** Fully rigid with no position bias: the relax pass of a rigid row. */
        val RIGID_RELAX = Softness(0.0, 1.0, 0.0)
    }
}

/**
 * Internal joint record. Body A is the tackler / blocker / first body, B the carrier / defender / second body.
 * All rows act at the body centers except the weld, whose A anchor sits at B's center in A's frame.
 * Accumulated impulses persist across ticks (warm start), except the grapple's drag.
 */
internal class Joint(
    @JvmField val id: Int,
    @JvmField val type: JointType,
    @JvmField val a: Body,
    @JvmField val b: Body,
    @JvmField val restLength: Double,
    /** The def's spring, or null for a rigid row. */
    @JvmField val soft: Softness?,
) {
    // Grapple
    @JvmField var breakForce = Double.POSITIVE_INFINITY
    @JvmField var dragForce = 0.0

    // Block
    @JvmField var pushForceA = 0.0
    @JvmField var pushForceB = 0.0
    @JvmField var shedThreshold = Double.POSITIVE_INFINITY
    @JvmField var driveSpeed = 1.0
    @JvmField var pushA = 0.0
    @JvmField var pushB = 0.0

    // Weld: B's center in A's frame, and the relative angle.
    @JvmField var localX = 0.0
    @JvmField var localY = 0.0
    @JvmField var refAngle = 0.0

    // Accumulated impulses (N·s).
    @JvmField var axial = 0.0
    @JvmField var lateral = 0.0
    @JvmField var dragX = 0.0
    @JvmField var dragY = 0.0
    @JvmField var linX = 0.0
    @JvmField var linY = 0.0
    @JvmField var angular = 0.0

    // Prepared each phase.
    @JvmField var solve = false
    @JvmField var ux = 1.0
    @JvmField var uy = 0.0
    @JvmField var len = 0.0
    @JvmField var axialMass = 0.0
    @JvmField var rAx = 0.0
    @JvmField var rAy = 0.0
    @JvmField var k11 = 0.0
    @JvmField var k12 = 0.0
    @JvmField var k22 = 0.0
    @JvmField var angularMass = 0.0

    /** Set when the solver needed more than the break force / shed threshold this tick. */
    @JvmField var broken = false

    private val inv = 1.0 / WorldConfig.DT

    fun force(): Double = when (type) {
        JointType.Grapple -> if (axial < 0.0) -axial * inv else 0.0
        JointType.Block, JointType.Distance -> DMath.abs(axial) * inv
        JointType.Weld -> DMath.hypot2(linX, linY) * inv
    }

    fun lateralForce(): Double = if (type == JointType.Block) DMath.abs(lateral) * inv else 0.0

    fun limit(): Double = when (type) {
        JointType.Grapple -> breakForce
        JointType.Block -> shedThreshold
        else -> Double.POSITIVE_INFINITY
    }

    fun load(): Double = when (type) {
        JointType.Grapple -> force() / breakForce
        JointType.Block -> lateralForce() / shedThreshold
        else -> 0.0
    }

    /**
     * Ground-reaction pushes of an engaged block, added to the bodies' forces before velocity integration.
     * Each side drives toward the other, scaled by leverage and tapered by its own speed toward the other.
     */
    fun applyPushes() {
        var dx = b.x - a.x
        var dy = b.y - a.y
        val l = DMath.hypot2(dx, dy)
        if (l > MIN_LENGTH) { dx /= l; dy /= l } else { dx = 1.0; dy = 0.0 }
        pushA = push(a, pushForceA, dx, dy)
        pushB = push(b, pushForceB, -dx, -dy)
        if (a.invMass > 0.0) { a.fx += pushA * dx; a.fy += pushA * dy }
        if (b.invMass > 0.0) { b.fx -= pushB * dx; b.fy -= pushB * dy }
    }

    private fun push(p: Body, max: Double, dx: Double, dy: Double): Double {
        if (max == 0.0) return 0.0
        val leverage = DMath.cos(p.angle) * dx + DMath.sin(p.angle) * dy
        if (leverage <= 0.0) return 0.0
        val taper = DMath.clamp(1.0 - (p.vx * dx + p.vy * dy) / driveSpeed, 0.0, 1.0)
        return max * leverage * taper
    }

    /** Computes axes and effective masses from the current positions. */
    fun prepare() {
        val mA = a.invMass; val mB = b.invMass
        val iA = a.invInertia; val iB = b.invInertia
        solve = (mA + mB) > 0.0 && (a.active || b.active)
        if (!solve) return
        var dx = b.x - a.x
        var dy = b.y - a.y
        len = DMath.hypot2(dx, dy)
        if (len > MIN_LENGTH) { dx /= len; dy /= len } else { dx = 1.0; dy = 0.0 }
        ux = dx; uy = dy
        axialMass = 1.0 / (mA + mB)
        if (type == JointType.Weld) {
            val c = DMath.cos(a.angle); val s = DMath.sin(a.angle)
            rAx = c * localX - s * localY
            rAy = s * localX + c * localY
            val m = mA + mB
            val a11 = m + iA * rAy * rAy
            val a12 = -iA * rAx * rAy
            val a22 = m + iA * rAx * rAx
            val det = a11 * a22 - a12 * a12
            // Inverse of the 2×2 point-constraint mass matrix.
            k11 = a22 / det; k12 = -a12 / det; k22 = a11 / det
            angularMass = if (iA + iB > 0.0) 1.0 / (iA + iB) else 0.0
        }
    }

    fun warmStart() {
        if (!solve) return
        when (type) {
            JointType.Grapple, JointType.Distance -> applyLinear(axial * ux, axial * uy)
            JointType.Block -> applyLinear(axial * ux - lateral * uy, axial * uy + lateral * ux)
            JointType.Weld -> {
                applyAngular(angular)
                applyWeldLinear(linX, linY)
            }
        }
    }

    /**
     * One sequential-impulse pass. While solving velocities ([relax] false) rigid rows use [rigid], the joint
     * softness; while relaxing they are fully rigid with no position bias. Springs from the def are physical forces,
     * not position fixes, so they act only while solving velocities (applying them again in relax overshoots).
     */
    fun solveVelocity(rigid: Softness, relax: Boolean) {
        if (!solve) return
        when (type) {
            JointType.Grapple -> {
                if (!relax) solveAxial(soft!!, -breakForce * WorldConfig.DT, 0.0)
                if (dragForce > 0.0) solveDrag()
            }
            JointType.Block -> {
                if (!relax) solveAxial(soft!!, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
                solveLateral()
            }
            JointType.Distance -> {
                val s = soft
                if (s == null) solveAxial(if (relax) Softness.RIGID_RELAX else rigid, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
                else if (!relax) solveAxial(s, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
            }
            JointType.Weld -> {
                val s = soft
                if (s == null) solveWeld(if (relax) Softness.RIGID_RELAX else rigid)
                else if (!relax) solveWeld(s)
            }
        }
    }

    private fun solveAxial(s: Softness, lo: Double, hi: Double) {
        val vr = (b.vx - a.vx) * ux + (b.vy - a.vy) * uy
        val c = DMath.hypot2(b.x - a.x, b.y - a.y) - restLength
        val old = axial
        val impulse = -s.massScale * axialMass * (vr + s.biasRate * c) - s.impulseScale * old
        val acc = DMath.clamp(old + impulse, lo, hi)
        axial = acc
        val d = acc - old
        applyLinear(d * ux, d * uy)
    }

    /** Rigid velocity row across the joint line, capped at the shed threshold. */
    private fun solveLateral() {
        val tx = -uy; val ty = ux
        val vt = (b.vx - a.vx) * tx + (b.vy - a.vy) * ty
        val cap = shedThreshold * WorldConfig.DT
        val old = lateral
        val acc = DMath.clamp(old - axialMass * vt, -cap, cap)
        lateral = acc
        val d = acc - old
        applyLinear(d * tx, d * ty)
    }

    /** Friction of the carrier against the ground, up to dragForce·dt per tick, toward zero velocity. */
    private fun solveDrag() {
        val m = b.invMass
        if (m == 0.0) return
        var px = dragX - b.vx / m
        var py = dragY - b.vy / m
        val max = dragForce * WorldConfig.DT
        val l = DMath.hypot2(px, py)
        if (l > max) { px *= max / l; py *= max / l }
        val dx = px - dragX; val dy = py - dragY
        dragX = px; dragY = py
        b.vx += m * dx; b.vy += m * dy
    }

    private fun solveWeld(s: Softness) {
        if (angularMass > 0.0) {
            val c = DMath.wrapPi(b.angle - a.angle - refAngle)
            val cdot = b.angularVelocity - a.angularVelocity
            val old = angular
            val impulse = -s.massScale * angularMass * (cdot + s.biasRate * c) - s.impulseScale * old
            angular = old + impulse
            applyAngular(impulse)
        }
        val cx = b.x - (a.x + rAx)
        val cy = b.y - (a.y + rAy)
        val vx = b.vx - (a.vx - a.angularVelocity * rAy)
        val vy = b.vy - (a.vy + a.angularVelocity * rAx)
        val bx = vx + s.biasRate * cx
        val by = vy + s.biasRate * cy
        val ix = -s.massScale * (k11 * bx + k12 * by) - s.impulseScale * linX
        val iy = -s.massScale * (k12 * bx + k22 * by) - s.impulseScale * linY
        linX += ix; linY += iy
        applyWeldLinear(ix, iy)
    }

    /** Impulse (px, py) on B at its center, the opposite on A at its center. */
    private fun applyLinear(px: Double, py: Double) {
        a.vx -= a.invMass * px; a.vy -= a.invMass * py
        b.vx += b.invMass * px; b.vy += b.invMass * py
    }

    private fun applyWeldLinear(px: Double, py: Double) {
        a.vx -= a.invMass * px; a.vy -= a.invMass * py
        a.angularVelocity -= a.invInertia * (rAx * py - rAy * px)
        b.vx += b.invMass * px; b.vy += b.invMass * py
    }

    private fun applyAngular(impulse: Double) {
        a.angularVelocity -= a.invInertia * impulse
        b.angularVelocity += b.invInertia * impulse
    }

    /** After the solver: did the grip or the block give way? The clamps leave a saturated impulse exactly at its cap. */
    fun checkBreak(): Boolean {
        if (!solve) return false
        broken = when (type) {
            JointType.Grapple -> breakForce != Double.POSITIVE_INFINITY && -axial >= breakForce * WorldConfig.DT
            JointType.Block -> shedThreshold != Double.POSITIVE_INFINITY && DMath.abs(lateral) >= shedThreshold * WorldConfig.DT
            else -> false
        }
        return broken
    }

    /** Called at the start of each tick: drag is friction and starts from zero every tick. */
    fun beginTick() {
        dragX = 0.0; dragY = 0.0
    }

    companion object {
        const val MIN_LENGTH = 1e-9
    }
}
