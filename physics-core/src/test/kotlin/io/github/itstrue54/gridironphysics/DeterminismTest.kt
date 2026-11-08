package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.SplitMix64
import org.junit.jupiter.api.Assertions.assertArrayEquals
import kotlin.test.Test
import kotlin.test.assertNotEquals

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

    @Test fun differentSeedDifferentHash() {
        assertNotEquals(
            randomWorld(1L).also { it.step() }.stateHash(),
            randomWorld(2L).also { it.step() }.stateHash(),
        )
    }
}
