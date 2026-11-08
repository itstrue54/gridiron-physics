package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.SplitMix64
import org.junit.jupiter.api.Assertions.assertArrayEquals
import kotlin.test.Test
import kotlin.test.assertNotEquals
import kotlin.test.assertTrue

class DeterminismTest {
    /** 22 dynamic players with random state plus one kinematic official, all drawn from [seed]. */
    private fun randomWorld(seed: Long): World {
        val r = SplitMix64(seed)
        val w = World(WorldConfig(), FieldConfig.PRO, seed)
        repeat(22) {
            w.createBody(
                BodyDef(
                    x = r.nextDouble(20.0, 90.0), y = r.nextDouble(5.0, 44.0),
                    angle = r.nextDouble(-3.0, 3.0),
                    vx = r.nextDouble(-9.0, 9.0), vy = r.nextDouble(-9.0, 9.0),
                    angularVelocity = r.nextDouble(-2.0, 2.0),
                    mass = r.nextDouble(80.0, 160.0),
                    linearDamping = r.nextDouble(0.0, 1.5), angularDamping = r.nextDouble(0.0, 1.5),
                    shape = ShapeDef.Circle(r.nextDouble(0.30, 0.38)),
                ),
            )
        }
        w.createBody(BodyDef(type = BodyType.Kinematic, y = -2.0, vx = 1.2))
        return w
    }

    @Test fun sameSeedSameHashEveryTick() {
        fun run(): LongArray {
            val w = randomWorld(seed = 1234L)
            return LongArray(600) { w.step(); w.stateHash() }
        }
        assertArrayEquals(run(), run())
    }

    /** A crowd crashing together in a small box, with every shape kind, so the solver and sleeping run hard. */
    private fun crashWorld(seed: Long): World {
        val r = SplitMix64(seed)
        val w = World(WorldConfig(), FieldConfig.PRO, seed)
        repeat(20) {
            val x = r.nextDouble(45.0, 60.0); val y = r.nextDouble(15.0, 30.0)
            w.createBody(
                BodyDef(
                    x = x, y = y, vx = (52.5 - x) * 0.8 + r.nextDouble(-1.0, 1.0), vy = (22.5 - y) * 0.8,
                    mass = r.nextDouble(80.0, 160.0), linearDamping = 0.5, angularDamping = 0.5,
                    shape = ShapeDef.Circle(r.nextDouble(0.30, 0.38)),
                    categoryBits = if (it % 2 == 0) Category.HOME else Category.AWAY,
                ),
            )
        }
        w.createBody(BodyDef(x = 50.0, y = 22.0, vx = 3.0, mass = 100.0, shape = ShapeDef.Capsule(0.6, 0.3), material = Material.DOWNED_PLAYER))
        w.createBody(BodyDef(x = 55.0, y = 23.0, mass = 250.0, linearDamping = 2.0, shape = ShapeDef.box(0.6, 1.2), material = Material.STRUCTURE))
        w.createBody(BodyDef(type = BodyType.Static, x = 52.0, y = 18.0, shape = ShapeDef.Polygon(doubleArrayOf(-1.0, -0.5, 1.0, -0.5, 0.0, 0.8)), material = Material.STRUCTURE))
        w.createBody(BodyDef(type = BodyType.Kinematic, x = 40.0, y = 22.5, vx = 2.0, categoryBits = Category.OFFICIAL))
        return w
    }

    @Test fun crashSameSeedSameHashEveryTick() {
        fun run(): LongArray {
            val w = crashWorld(seed = 77L)
            return LongArray(600) { w.step(); w.stateHash() }
        }
        val a = run()
        assertArrayEquals(a, run())
        val w = crashWorld(77L)
        var contacts = 0
        repeat(600) { w.step(); contacts += w.contactCount }
        assertTrue(contacts > 100, "the crowd must actually collide: $contacts")
        val out = IntArray(64)
        val n = w.bodyIds(out)
        for (i in 0 until n) { val b = w.body(BodyId(out[i])); assertTrue(b.x.isFinite() && b.y.isFinite() && b.vx.isFinite()) }
    }

    @Test fun differentSeedDifferentHash() {
        assertNotEquals(
            randomWorld(1L).also { it.step() }.stateHash(),
            randomWorld(2L).also { it.step() }.stateHash(),
        )
    }
}
