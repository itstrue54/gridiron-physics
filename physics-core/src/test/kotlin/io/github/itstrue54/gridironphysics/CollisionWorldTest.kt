package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** World-level collision behavior: walls, sensors, filtering, materials, solver, contacts, sleeping. */
class CollisionWorldTest {
    private fun world(cfg: WorldConfig = WorldConfig()) = World(cfg, FieldConfig.PRO, 1L)

    private fun player(w: World, x: Double, y: Double, vx: Double = 0.0, vy: Double = 0.0, mass: Double = 100.0, r: Double = 0.34) =
        w.createBody(BodyDef(x = x, y = y, vx = vx, vy = vy, mass = mass, shape = ShapeDef.Circle(r)))

    // ------------------------------------------------------------ runoff walls

    @Test fun runoffWallsKeepBodiesInTheWorld() {
        val f = FieldConfig.PRO
        val w = world()
        val down = player(w, 50.0, 2.0, vy = -10.0)
        val up = player(w, 60.0, f.widthM - 2.0, vy = 10.0)
        val back = player(w, 2.0, 20.0, vx = -10.0)
        val fwd = player(w, f.lengthM - 2.0, 20.0, vx = 10.0)
        repeat(180) { w.step() }
        val r = 0.34
        assertTrue(w.body(down).y >= -f.runoffM + r - 0.01, "near wall: ${w.body(down).y}")
        assertTrue(w.body(up).y <= f.widthM + f.runoffM - r + 0.01, "far wall: ${w.body(up).y}")
        assertTrue(w.body(back).x >= -f.runoffM + r - 0.01, "home end wall: ${w.body(back).x}")
        assertTrue(w.body(fwd).x <= f.lengthM + f.runoffM - r + 0.01, "away end wall: ${w.body(fwd).x}")
        assertTrue(w.body(down).vy >= 0.0, "bounced or stopped")
    }

    @Test fun runoffWallsCanBeTurnedOff() {
        val w = world(WorldConfig(runoffWalls = false))
        val id = player(w, 50.0, 2.0, vy = -10.0)
        repeat(180) { w.step() }
        assertTrue(w.body(id).y < -FieldConfig.PRO.runoffM)
    }

    @Test fun wallsAreNotBodies() {
        val w = world()
        assertEquals(0, w.bodyCount)
    }

    @Test fun wallContactIsReported() {
        val w = world()
        player(w, 50.0, -5.65, vy = -2.0) // 0.01 m from the wall: inside the speculative margin
        w.step()
        assertEquals(1, w.contactCount)
        val c = w.contact(0)
        assertEquals(ContactView.WALL_NEAR, c.wall)
        assertEquals(0.0, c.normalX, 1e-12); assertEquals(-1.0, c.normalY, 1e-12)
    }

    // ------------------------------------------------------------ sensors and filtering

    @Test fun sensorShapeDetectsButDoesNotPush() {
        val w = world()
        w.createBody(BodyDef(type = BodyType.Static, x = 52.0, y = 20.0, shape = ShapeDef.Circle(1.0, sensor = true)))
        val p = player(w, 50.0, 20.0, vx = 5.0)
        var sawSensor = false
        repeat(60) {
            w.step()
            for (i in 0 until w.contactCount) if (w.contact(i).sensor && w.contact(i).touching) sawSensor = true
        }
        assertTrue(sawSensor)
        assertEquals(5.0, w.body(p).vx, 1e-12)
        assertEquals(0.0, w.body(p).vy, 1e-12)
    }

    @Test fun maskBitsSkipThePair() {
        val w = world()
        val noHome = Category.ALL and Category.HOME.inv()
        val a = w.createBody(BodyDef(x = 50.0, y = 20.0, vx = 3.0, categoryBits = Category.HOME, maskBits = noHome))
        val b = w.createBody(BodyDef(x = 51.0, y = 20.0, vx = -3.0, categoryBits = Category.HOME, maskBits = noHome))
        repeat(30) { w.step(); assertEquals(0, w.contactCount) }
        assertEquals(3.0, w.body(a).vx, 1e-12); assertEquals(-3.0, w.body(b).vx, 1e-12)
    }

    @Test fun filterNeedsBothSidesToAgree() {
        val w = world()
        // a would collide with b, but b's mask excludes a's category.
        val a = w.createBody(BodyDef(x = 50.0, y = 20.0, vx = 3.0, categoryBits = Category.HOME, maskBits = Category.ALL))
        w.createBody(BodyDef(x = 51.0, y = 20.0, vx = -3.0, categoryBits = Category.AWAY, maskBits = Category.ALL and Category.HOME.inv()))
        repeat(30) { w.step() }
        assertEquals(3.0, w.body(a).vx, 1e-12)
    }

    @Test fun ballAgainstPlayerIsSensorOnly() {
        val w = world()
        val player = w.createBody(BodyDef(x = 51.0, y = 20.0, categoryBits = Category.AWAY))
        val ball = w.createBody(
            BodyDef(x = 50.0, y = 20.0, vx = 6.0, mass = 0.41, shape = ShapeDef.Circle(0.09), categoryBits = Category.BALL, material = Material.BALL),
        )
        var sawSensor = false
        repeat(30) {
            w.step()
            for (i in 0 until w.contactCount) if (w.contact(i).sensor && w.contact(i).touching) sawSensor = true
        }
        assertTrue(sawSensor, "possession is game logic; the engine still reports the overlap")
        assertEquals(6.0, w.body(ball).vx, 1e-12)
        assertEquals(0.0, w.body(player).vx, 1e-12)
    }

    // ------------------------------------------------------------ materials, restitution, friction

    @Test fun materialTableMatchesSpecDefaults() {
        val t = MaterialTable.DEFAULT
        fun check(a: Int, b: Int, mu: Double, e: Double) {
            assertEquals(mu, t.friction(a, b)); assertEquals(e, t.restitution(a, b))
            assertEquals(mu, t.friction(b, a)); assertEquals(e, t.restitution(b, a))
        }
        check(Material.PLAYER, Material.PLAYER, 0.6, 0.02)
        check(Material.PLAYER, Material.DOWNED_PLAYER, 0.8, 0.0)
        check(Material.PLAYER, Material.STRUCTURE, 0.4, 0.1)
        check(Material.BALL, Material.TURF, 0.5, 0.5)
        check(Material.BALL, Material.STRUCTURE, 0.3, 0.6)
        check(Material.BALL, Material.PLAYER, 0.3, 0.3)
    }

    @Test fun materialTableWithIsSymmetricAndDoesNotMutate() {
        val t = MaterialTable.DEFAULT.with(Material.STRUCTURE, Material.PLAYER, friction = 0.0, restitution = 1.0)
        assertEquals(0.0, t.friction(Material.PLAYER, Material.STRUCTURE))
        assertEquals(1.0, t.restitution(Material.PLAYER, Material.STRUCTURE))
        assertEquals(0.4, MaterialTable.DEFAULT.friction(Material.PLAYER, Material.STRUCTURE))
    }

    private fun bounce(closingSpeed: Double): Double {
        val mats = MaterialTable.DEFAULT.with(Material.PLAYER, Material.PLAYER, friction = 0.0, restitution = 1.0)
        val w = world(WorldConfig(materials = mats))
        val a = player(w, 50.0, 20.0, vx = closingSpeed / 2)
        val b = player(w, 51.0, 20.0, vx = -closingSpeed / 2)
        repeat(90) { w.step() }
        return w.body(b).vx - w.body(a).vx // separating speed after the hit
    }

    @Test fun restitutionOnlyAboveThreshold() {
        assertEquals(4.0, bounce(4.0), 1e-9)
        assertEquals(0.0, bounce(0.8), 1e-3)
    }

    @Test fun frictionSlowsTangentialSliding() {
        fun slide(mu: Double): Double {
            val mats = MaterialTable.DEFAULT.with(Material.PLAYER, Material.STRUCTURE, friction = mu, restitution = 0.0)
            val w = world(WorldConfig(materials = mats))
            w.createBody(BodyDef(type = BodyType.Static, x = 50.0, y = 18.0, shape = ShapeDef.box(10.0, 1.0), material = Material.STRUCTURE))
            val p = player(w, 45.0, 19.5, vx = 5.0, vy = -5.0)
            repeat(30) { w.step() }
            return w.body(p).vx
        }
        assertEquals(5.0, slide(0.0), 1e-9)
        assertTrue(slide(0.4) < 4.0, "friction should take tangential speed: ${slide(0.4)}")
    }

    // ------------------------------------------------------------ solver

    @Test fun kinematicPushesAChainWithoutDeepPenetration() {
        val w = world()
        val r = 0.34
        val chain = (0 until 5).map { player(w, 50.0 + 2 * r * (it + 1), 20.0) }
        w.createBody(BodyDef(type = BodyType.Kinematic, x = 50.0 - 0.05, y = 20.0, vx = 2.0, shape = ShapeDef.Circle(r)))
        var maxDepth = 0.0
        repeat(180) {
            w.step()
            for (i in 0 until w.contactCount) {
                val c = w.contact(i)
                for (k in 0 until c.pointCount) maxDepth = maxOf(maxDepth, -c.separation(k))
            }
        }
        assertTrue(maxDepth < 0.02, "max penetration $maxDepth")
        assertTrue(w.body(chain.last()).x > 50.0 + 10 * r + 4.0, "the chain is driven forward")
    }

    @Test fun contactsAreSortedByBodyIdPair() {
        val w = world()
        repeat(6) { player(w, 50.0 + 0.4 * (it % 3), 20.0 + 0.4 * (it / 3)) }
        w.step()
        assertTrue(w.contactCount >= 5)
        var lastA = -1; var lastB = -1
        for (i in 0 until w.contactCount) {
            val c = w.contact(i)
            val a = c.bodyA.value; val b = c.bodyB.value
            assertTrue(a < b, "lower id first")
            assertTrue(a > lastA || (a == lastA && b > lastB), "sorted by (a, b)")
            lastA = a; lastB = b
        }
    }

    @Test fun overlappingBodiesArePushedApartSoftly() {
        val w = world()
        val a = player(w, 50.0, 20.0)
        val b = player(w, 50.4, 20.0)
        var maxSpeed = 0.0
        repeat(120) {
            w.step()
            maxSpeed = maxOf(maxSpeed, kotlin.math.abs(w.body(a).vx), kotlin.math.abs(w.body(b).vx))
        }
        assertTrue(w.body(b).x - w.body(a).x > 0.68 - 0.01, "pushed apart: ${w.body(b).x - w.body(a).x}")
        assertTrue(maxSpeed <= WorldConfig().maxPushoutSpeed + 1e-9, "push-out is capped: $maxSpeed")
    }

    @Test fun hitReportsNormalImpulse() {
        val w = world()
        player(w, 50.0, 20.0, vx = 5.0)
        player(w, 51.0, 20.0)
        var maxImpulse = 0.0
        repeat(30) { w.step(); for (i in 0 until w.contactCount) maxImpulse = maxOf(maxImpulse, w.contact(i).normalImpulse(0)) }
        assertTrue(maxImpulse > 100.0, "a 5 m/s hit between 100 kg bodies is a few hundred N·s: $maxImpulse")
    }

    @Test fun capsuleAndPolygonBodiesCollide() {
        val w = world()
        val sled = w.createBody(BodyDef(x = 52.0, y = 20.0, mass = 200.0, shape = ShapeDef.box(0.5, 1.0), material = Material.STRUCTURE))
        val diver = w.createBody(BodyDef(x = 50.0, y = 20.0, vx = 6.0, mass = 100.0, shape = ShapeDef.Capsule(0.6, 0.3)))
        repeat(60) { w.step() }
        assertTrue(w.body(sled).vx > 0.5, "the sled is driven: ${w.body(sled).vx}")
        assertTrue(w.body(diver).x < w.body(sled).x, "no tunneling")
    }

    // ------------------------------------------------------------ sleeping

    @Test fun restingBodySleepsAfterHalfSecond() {
        val w = world()
        val id = player(w, 50.0, 20.0, vx = 0.01)
        repeat(29) { w.step() }
        assertTrue(w.body(id).awake)
        repeat(3) { w.step() }
        assertFalse(w.body(id).awake)
        assertEquals(0.0, w.body(id).vx)
        val x = w.body(id).x
        repeat(60) { w.step() }
        assertEquals(x, w.body(id).x)
    }

    @Test fun fastBodyStaysAwake() {
        val w = world()
        val id = player(w, 50.0, 20.0, vx = 1.0)
        repeat(60) { w.step() }
        assertTrue(w.body(id).awake)
    }

    @Test fun contactWakesASleeper() {
        val w = world()
        val sleeper = player(w, 60.0, 20.0)
        repeat(40) { w.step() }
        assertFalse(w.body(sleeper).awake)
        val runner = player(w, 55.0, 20.0, vx = 6.0)
        repeat(60) { w.step() }
        assertTrue(w.body(sleeper).x > 60.0 + 0.1, "the sleeper is knocked forward: ${w.body(sleeper).x}")
        assertTrue(w.body(runner).x < w.body(sleeper).x)
    }

    @Test fun touchingRestingBodiesSleepTogether() {
        val w = world()
        val a = player(w, 50.0, 20.0)
        val b = player(w, 50.68, 20.0)
        repeat(40) { w.step() }
        assertFalse(w.body(a).awake); assertFalse(w.body(b).awake)
    }

    @Test fun kinematicNeverSleepsWhileMoving() {
        val w = world()
        val k = w.createBody(BodyDef(type = BodyType.Kinematic, x = 50.0, y = 20.0, vx = 0.01))
        repeat(60) { w.step() }
        assertTrue(w.body(k).awake)
        assertEquals(50.0 + 0.01, w.body(k).x, 1e-9)
    }
}
