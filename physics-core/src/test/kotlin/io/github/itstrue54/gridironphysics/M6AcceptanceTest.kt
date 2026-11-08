package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.DMath
import io.github.itstrue54.gridironphysics.math.SplitMix64
import io.github.itstrue54.gridironphysics.math.Vec3
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** Spec Testing table, M6 rows: Vacuum ball range, Drag sanity, Launch solver, No tunneling. */
class M6AcceptanceTest {
    private val f = FieldConfig.PRO

    /** Steps until the ball first touches the ground and returns that bounce event. */
    private fun firstBounce(w: World, maxTicks: Int = 1200): Ev {
        repeat(maxTicks) {
            w.step()
            w.drain().firstOrNull { it.type == EventType.BallBounce }?.let { return it }
        }
        error("the ball never landed in $maxTicks ticks (state ${w.ball().state}, z ${w.ball().z})")
    }

    private fun range(ball: BallConfig, mode: FlightMode): Double {
        val w = World(WorldConfig(ball = ball), f, 1L)
        val v = 25.0 / DMath.sqrt(2.0)
        w.launchBall(Vec3(10.0, 24.0, 0.0), Vec3(v, 0.0, v), mode)
        return firstBounce(w).px - 10.0
    }

    /** 25 m/s at 45°, drag off: lands at 63.71 m ±1 cm (v²/g = 63.7105 m). */
    @Test fun vacuumBallRange() {
        val r = range(BallConfig(drag = false), FlightMode.Spiral)
        assertEquals(25.0 * 25.0 / 9.81, r, 0.01)
        assertEquals(63.71, r, 0.01)
    }

    /** Same launch, drag on: range shorter than vacuum; spiral range > wobble range (> end over end). */
    @Test fun dragSanity() {
        val vacuum = range(BallConfig(drag = false), FlightMode.Spiral)
        val spiral = range(BallConfig(), FlightMode.Spiral)
        val wobble = range(BallConfig(), FlightMode.Wobble)
        val endOverEnd = range(BallConfig(), FlightMode.EndOverEnd)
        assertTrue(spiral < vacuum, "spiral $spiral < vacuum $vacuum")
        assertTrue(spiral > wobble, "spiral $spiral > wobble $wobble")
        assertTrue(wobble > endOverEnd, "wobble $wobble > end over end $endOverEnd")
    }

    /**
     * 500 random targets 5–55 m away, in random wind: the ball lands within 0.25 m of the target. Even targets use the
     * apex solver (random arc height, every flight mode), odd ones the speed solver (random speed, low or high arc).
     */
    @Test fun launchSolver() {
        val rng = SplitMix64(606L)
        val w = World(WorldConfig(), f, 2L)
        val from = Vec3()
        val target = Vec3()
        val v = Vec3()
        var worst = 0.0
        for (i in 0 until 500) {
            w.setWind(rng.nextDouble(-6.0, 6.0), rng.nextDouble(-6.0, 6.0))
            // Kept 55 m short of the away goal posts, which the solver does not know about.
            from.set(rng.nextDouble(30.0, 50.0), rng.nextDouble(15.0, 34.0), rng.nextDouble(0.0, 2.2))
            val d = rng.nextDouble(5.0, 55.0)
            val a = rng.nextDouble(0.0, 2 * DMath.PI)
            target.set(from.x + d * DMath.cos(a), from.y + d * DMath.sin(a), 0.0)
            var mode = FlightMode.entries[((rng.nextLong() ushr 1) % 3).toInt()]
            if (i % 2 == 0) {
                val apex = from.z + rng.nextDouble(1.0, 20.0)
                w.launchVelocityFor(from, target, apex, mode, v)
            } else {
                if (mode == FlightMode.EndOverEnd) mode = FlightMode.Wobble // a passer's speed range doesn't reach 55 m end over end
                val m = mode
                val speed = rng.nextDouble(31.0, 34.0)
                val high = rng.nextDouble() < 0.5
                assertTrue(w.launchVelocityFor(from, target, speed, high, m, v), "target $i at $d m reachable at $speed m/s ($m)")
                assertEquals(speed, DMath.sqrt(v.x * v.x + v.y * v.y + v.z * v.z), 1e-9)
            }
            w.launchBall(from, v, mode)
            val b = firstBounce(w)
            val miss = DMath.hypot2(b.px - target.x, b.py - target.y)
            if (miss > worst) worst = miss
            assertTrue(miss < 0.25, "target $i ($d m, $mode): missed by $miss m")
        }
        assertTrue(worst < 0.25, "worst miss $worst")
    }

    /**
     * The airborne ball at 30 m/s into an upright from 1,000 random directions: zero pass-throughs. Each shot is
     * aimed at a random point of the right upright of the away goal post (sometimes off-center, a glancing hit),
     * from 3–10 m away. A pass-through is a shot with no HitPost, or a tick where the ball's center is closer to the
     * upright's axis than the two radii.
     */
    @Test fun noTunneling() {
        val rng = SplitMix64(30L)
        val px = f.lengthM
        val py = f.widthM / 2 - f.uprightSpacingM / 2
        val reach = BallConfig().radius + f.uprightRadiusM
        val from = Vec3(); val v = Vec3()
        var passes = 0
        for (i in 0 until 1000) {
            val w = World(WorldConfig(), f, i.toLong())
            val az = rng.nextDouble(0.0, 2 * DMath.PI)
            val el = rng.nextDouble(-DMath.PI / 6, DMath.PI / 6)
            val d = rng.nextDouble(3.0, 10.0)
            val hz = rng.nextDouble(5.0, 12.0)
            val off = rng.nextDouble(-0.9, 0.9) * reach // perpendicular miss distance of the aim line
            val ux = DMath.cos(el) * DMath.cos(az); val uy = DMath.cos(el) * DMath.sin(az); val uz = DMath.sin(el)
            // Aim point: on the axis, nudged sideways (perpendicular to the horizontal direction).
            val ax = px - DMath.sin(az) * off; val ay = py + DMath.cos(az) * off
            from.set(ax - d * ux, ay - d * uy, hz - d * uz)
            val t = d / 30.0
            v.set(30.0 * ux, 30.0 * uy, 30.0 * uz + 0.5 * 9.81 * t) // lift the aim for gravity's drop
            w.launchBall(from, v, FlightMode.Spiral)
            var hit = false
            var through = false
            for (k in 0 until 60) {
                w.step()
                if (w.drain().any { it.type == EventType.KickResult && it.kick == KickResult.HitPost }) hit = true
                val b = w.ball()
                if (b.z in f.crossbarHeightM..(f.crossbarHeightM + f.uprightHeightAboveCrossbarM)) {
                    val dd = DMath.hypot2(b.x - px, b.y - py)
                    if (dd < reach - 1e-6) through = true
                }
                if (hit && k > 20) break
            }
            if (!hit || through) passes++
        }
        assertEquals(0, passes, "pass-throughs out of 1000")
    }
}
