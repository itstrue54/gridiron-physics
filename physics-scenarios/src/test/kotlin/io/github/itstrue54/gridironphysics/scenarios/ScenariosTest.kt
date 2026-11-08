package io.github.itstrue54.gridironphysics.scenarios

import io.github.itstrue54.gridironphysics.BodyId
import io.github.itstrue54.gridironphysics.BodyType
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.World
import io.github.itstrue54.gridironphysics.math.DMath
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ScenariosTest {
    private fun ids(w: World): List<BodyId> {
        val out = IntArray(64)
        val n = w.bodyIds(out)
        return (0 until n).map { BodyId(out[it]) }
    }

    @Test fun driftHas22PlayersOneOfficialOneProp() {
        val w = Scenarios.drift.build()
        val ids = ids(w)
        assertEquals(24, ids.size)
        assertEquals(11, ids.count { w.body(it).userData == Tags.HOME })
        assertEquals(11, ids.count { w.body(it).userData == Tags.AWAY })
        assertEquals(1, ids.count { w.body(it).userData == Tags.OFFICIAL && w.body(it).type == BodyType.Kinematic })
        assertEquals(1, ids.count { w.body(it).userData == Tags.PROP && w.body(it).type == BodyType.Static })
    }

    @Test fun driftPlayersUseSpecRanges() {
        val w = Scenarios.drift.build()
        ids(w).filter { w.body(it).userData <= Tags.AWAY }.forEach {
            val b = w.body(it)
            assertTrue(b.mass in 80.0..160.0, "mass ${b.mass}")
            assertTrue(b.radius in 0.30..0.38, "radius ${b.radius}")
        }
    }

    @Test fun driftSlowsDownAndStaysFinite() {
        val w = Scenarios.drift.build()
        val ids = ids(w)
        fun maxPlayerSpeed() = ids.filter { w.body(it).userData <= Tags.AWAY }
            .maxOf { val b = w.body(it); DMath.hypot2(b.vx, b.vy) }
        val v0 = maxPlayerSpeed()
        assertTrue(v0 > 1.0, "players should start moving")
        repeat(600) { w.step() }
        assertTrue(maxPlayerSpeed() < v0 * 0.05)
        ids.forEach { val b = w.body(it); assertTrue(b.x.isFinite() && b.y.isFinite()) }
    }

    @Test fun buildIsDeterministic() {
        assertEquals(Scenarios.drift.build().stateHash(), Scenarios.drift.build().stateHash())
    }

    // ------------------------------------------------------------ M3: head-on tackle (locomotion)

    private fun wrap(a: Double): Double {
        var r = a % (2 * DMath.PI)
        if (r > DMath.PI) r -= 2 * DMath.PI
        if (r < -DMath.PI) r += 2 * DMath.PI
        return r
    }

    @Test fun headOnTackleRunsOnLocomotionNotCoasting() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        for (id in ids) {
            val b = w.body(id)
            assertTrue(b.athlete != null, "body ${id.value} has an athlete")
            assertEquals(0.0, b.linearDamping); assertEquals(0.0, b.angularDamping)
        }
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        assertTrue(w.body(lb).x - w.body(back).x >= 10.0, "they start far enough apart to see them run")
    }

    @Test fun headOnTackleDrivesThroughContact() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        var hitTick = -1
        var pushing = 0
        var hitX = 0.0
        for (t in 0 until 180) {
            w.step()
            var push = false
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                if (c.wall == io.github.itstrue54.gridironphysics.ContactView.NO_WALL && c.normalImpulse(0) > 0.0) push = true
            }
            if (push && hitTick < 0) { hitTick = t; hitX = (w.body(back).x + w.body(lb).x) / 2 }
            if (hitTick >= 0 && t in hitTick + 1..hitTick + 30 && push) pushing++
            if (hitTick >= 0 && t == hitTick + 30) {
                // Both still face where they are trying to go: the back downfield, the linebacker upfield.
                assertEquals(0.0, wrap(w.body(back).angle), 0.2, "back's heading")
                assertEquals(0.0, wrap(w.body(lb).angle - DMath.PI), 0.2, "linebacker's heading")
            }
        }
        assertTrue(hitTick >= 0, "they must collide")
        // A shove, not a bounce: they stay in pushing contact for a while, then drive on past each other.
        assertTrue(pushing >= 8, "they keep driving into each other after the hit: pushed on $pushing of 30 ticks")
        assertTrue(w.body(back).x > hitX + 3.0 && w.body(back).vx > 4.0, "the back drives on downfield: x ${w.body(back).x}, vx ${w.body(back).vx}")
        assertTrue(w.body(lb).x < hitX - 3.0 && w.body(lb).vx < -3.0, "the linebacker drives on upfield: x ${w.body(lb).x}, vx ${w.body(lb).vx}")
    }

    @Test fun headOnTackleIsA90KgBackVsA115KgLinebacker() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        assertEquals(2, ids.size)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        assertEquals(90.0, w.body(back).mass); assertEquals(115.0, w.body(lb).mass)
        assertTrue(w.body(back).vx > 8.0 && w.body(lb).vx < -6.0, "both at full speed, head-on")
    }

    @Test fun headOnTackleCollidesWithoutTunnelingAndConservesMomentum() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        fun px() = w.body(back).mass * w.body(back).vx + w.body(lb).mass * w.body(lb).vx
        var hit = false
        var maxDepth = 0.0
        var pBefore = px()
        var pAfter = 0.0
        val rSum = w.body(back).radius + w.body(lb).radius
        repeat(180) {
            val p = px()
            w.step()
            // On locomotion they shove past each other after the hit, so "no tunneling" means never through.
            val gap = DMath.hypot2(w.body(lb).x - w.body(back).x, w.body(lb).y - w.body(back).y)
            assertTrue(gap > rSum - 0.01, "centers $gap apart, radii sum $rSum")
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                if (c.normalImpulse(0) > 0.0 && !hit) { hit = true; pBefore = p; pAfter = px() }
                for (k in 0 until c.pointCount) maxDepth = maxOf(maxDepth, -c.separation(k))
            }
        }
        assertTrue(hit, "they must collide")
        assertTrue(maxDepth < 0.01, "max penetration $maxDepth")
        // Locomotion forces act during the impact tick (near zero at top speed), so allow a small tolerance.
        assertEquals(pBefore, pAfter, kotlin.math.abs(pBefore) * 0.02 + 5.0)
    }

    @Test fun headOnTackleHitsAtNearlyFullSpeed() {
        val w = Scenarios.headOnTackle.build()
        val ids = ids(w)
        val back = ids.single { w.body(it).userData == Tags.HOME }
        val lb = ids.single { w.body(it).userData == Tags.AWAY }
        var vBack = 0.0; var vLb = 0.0
        var hit = false
        repeat(120) {
            if (!hit) { vBack = w.body(back).vx; vLb = w.body(lb).vx }
            w.step()
            for (i in 0 until w.contactCount) if (w.contact(i).normalImpulse(0) > 0.0) hit = true
        }
        assertTrue(hit)
        assertTrue(vBack >= 0.95 * 9.0, "back should arrive near 9 m/s, was $vBack")
        assertTrue(vLb <= -0.95 * 7.0, "linebacker should arrive near 7 m/s, was $vLb")
    }

    @Test fun playersStopSpinningSoonAfterAHit() {
        // Coasting bodies only: players on locomotion hold their own heading (headOnTackleDrivesThroughContact).
        for (sc in listOf(Scenarios.contactLab)) {
            val w = sc.build()
            val ids = ids(w)
            // Each body is checked 0.5 s after the last tick it took an impulse.
            val lastHit = HashMap<Int, Int>()
            var checks = 0
            for (t in 0 until 400) {
                w.step()
                for (i in 0 until w.contactCount) {
                    val c = w.contact(i)
                    if (c.sensor || c.pointCount == 0 || c.normalImpulse(0) <= 0.0) continue
                    lastHit[c.bodyA.value] = t
                    if (c.wall == io.github.itstrue54.gridironphysics.ContactView.NO_WALL) lastHit[c.bodyB.value] = t
                }
                ids.filter { w.body(it).type == BodyType.Dynamic && lastHit[it.value] == t - 30 }.forEach {
                    checks++
                    val spin = kotlin.math.abs(w.body(it).angularVelocity)
                    assertTrue(spin < 0.3, "${sc.name}: body ${it.value} still spinning at $spin rad/s, 0.5 s after its last hit")
                }
            }
            assertTrue(checks > 0, "${sc.name} must have a hit")
        }
    }

    // ------------------------------------------------------------ M2: contact lab (shapes, walls, sleeping)

    @Test fun contactLabEveryInteractionActuallyHappens() {
        val w = Scenarios.contactLab.build()
        val ids = ids(w)
        val touched = HashSet<Pair<Int, Int>>()
        val walls = HashSet<Int>()
        repeat(300) {
            w.step()
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                if (!c.touching && c.normalImpulse(0) == 0.0) continue
                if (c.wall != io.github.itstrue54.gridironphysics.ContactView.NO_WALL) walls += c.bodyA.value
                else touched += c.bodyA.value to c.bodyB.value
            }
        }
        assertEquals(1, walls.size, "one runner hits the runoff wall")
        // Pairs that must touch: runner–sled, diver–downed player, runner–post base, the two angled players, runner–line sensor.
        assertTrue(touched.size >= 5, "expected at least 5 touching pairs, got ${touched.size}: $touched")
    }

    @Test fun contactLabUsesEveryShapeKind() {
        val w = Scenarios.contactLab.build()
        val shapes = ids(w).map { w.body(it).shape::class }.toSet()
        assertTrue(ShapeDef.Circle::class in shapes)
        assertTrue(ShapeDef.Capsule::class in shapes)
        assertTrue(ShapeDef.Polygon::class in shapes)
        assertTrue(ShapeDef.Segment::class in shapes)
    }

    @Test fun contactLabKeepsEveryoneInsideTheWorldAndSettles() {
        val w = Scenarios.contactLab.build()
        repeat(600) { w.step() }
        val f = w.field
        ids(w).forEach {
            val b = w.body(it)
            assertTrue(b.x > -f.runoffM && b.x < f.lengthM + f.runoffM && b.y > -f.runoffM && b.y < f.widthM + f.runoffM, "body ${it.value} left the world")
        }
        assertTrue(ids(w).filter { w.body(it).type == BodyType.Dynamic }.all { !w.body(it).awake }, "everything has come to rest after 10 s")
    }

    // ------------------------------------------------------------ M3: sideline run

    private class Sideline(val w: World, val carrier: BodyId, val pursuer: BodyId)

    private fun sideline(): Sideline {
        val w = Scenarios.sidelineRun.build()
        val ids = ids(w)
        assertEquals(2, ids.size)
        return Sideline(w, ids.single { w.body(it).userData == Tags.HOME }, ids.single { w.body(it).userData == Tags.AWAY })
    }

    @Test fun sidelineRunStartsWithTheCarrierAtTopSpeedAndThePursuerAtAnAngle() {
        val s = sideline()
        // BodyView is one shared instance per world: read each body's values before viewing the next.
        val c = s.w.body(s.carrier)
        val top = c.athlete!!.maxSpeed
        assertEquals(top, c.vx, 1e-12); assertEquals(0.0, c.vy)
        assertTrue(c.y < 5.0, "carrier runs near the sideline, y = ${c.y}")
        val cx = c.x; val cy = c.y
        val p = s.w.body(s.pursuer)
        assertTrue(p.athlete!!.maxSpeed < top, "the pursuer is slower, so he needs the angle")
        assertTrue(p.y > cy + 8.0 && p.x > cx, "pursuer starts infield and ahead")
        assertTrue(p.vy < -1.0 && kotlin.math.abs(p.vx) > 1.0, "pursuer runs at an angle: (${p.vx}, ${p.vy})")
    }

    @Test fun sidelineRunPursuerCutsTheCarrierOffNearTheSideline() {
        val s = sideline()
        val w = s.w
        val top = w.body(s.carrier).athlete!!.maxSpeed
        val y0 = w.body(s.carrier).y
        var hitTick = -1
        var hitY = 0.0
        var vAtHit = 0.0
        for (t in 0 until 300) {
            val v = w.body(s.carrier).vx
            w.step()
            if (hitTick < 0) {
                for (i in 0 until w.contactCount) {
                    val c = w.contact(i)
                    if (c.wall == io.github.itstrue54.gridironphysics.ContactView.NO_WALL && c.normalImpulse(0) > 0.0) {
                        hitTick = t; hitY = c.pointY(0); vAtHit = v
                    }
                }
                if (hitTick < 0) assertEquals(y0, w.body(s.carrier).y, 1e-9, "carrier runs a straight line until the hit")
                for (i in 0 until w.slipCount) assertTrue(w.slipBody(i) != s.carrier, "no slip running straight")
            }
        }
        assertTrue(hitTick in 0 until 150, "pursuer intercepts within 2.5 s, hit at tick $hitTick")
        assertTrue(hitY < 6.0, "the hit is near the sideline, y = $hitY")
        assertTrue(vAtHit >= 0.99 * top, "carrier is at top speed when hit: $vAtHit")
        val f = w.field
        for (id in listOf(s.carrier, s.pursuer)) {
            val b = w.body(id)
            assertTrue(b.x > -f.runoffM && b.x < f.lengthM + f.runoffM && b.y > -f.runoffM && b.y < f.widthM + f.runoffM, "body ${id.value} left the world")
        }
    }

    @Test fun sidelineRunIsDeterministic() {
        val a = Scenarios.sidelineRun.build(); val b = Scenarios.sidelineRun.build()
        repeat(240) {
            a.step(); b.step()
            assertEquals(a.stateHash(), b.stateHash())
        }
    }

    @Test fun registryListsDrift() {
        assertTrue(Scenarios.drift in Scenarios.all)
        assertTrue(Scenarios.sidelineRun in Scenarios.all)
        assertTrue(Scenarios.headOnTackle in Scenarios.all)
        assertTrue(Scenarios.contactLab in Scenarios.all)
        assertEquals(Scenarios.all.size, Scenarios.all.map { it.name }.toSet().size, "scenario names are unique")
    }
}
