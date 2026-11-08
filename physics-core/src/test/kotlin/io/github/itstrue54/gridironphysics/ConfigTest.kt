package io.github.itstrue54.gridironphysics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith

class ConfigTest {
    @Test fun proPresetMatchesSpec() {
        val f = FieldConfig.PRO
        assertEquals(109.728, f.lengthM)
        assertEquals(48.768, f.widthM)
        assertEquals(9.144, f.homeGoalLineX)
        assertEquals(100.584, f.awayGoalLineX, 1e-12)
        assertEquals(54.864, f.midfieldX, 1e-12)
        assertEquals(5.639, f.farHashY - f.nearHashY, 1e-12)
        assertEquals(3.048, f.crossbarHeightM)
        assertEquals(10.668, f.uprightHeightAboveCrossbarM)
        assertEquals(6.0, f.runoffM)
    }

    @Test fun collegePresetOnlyChangesHashes() {
        assertEquals(12.192, FieldConfig.COLLEGE.hashSpacingM)
        assertEquals(FieldConfig.PRO.uprightSpacingM, FieldConfig.COLLEGE.uprightSpacingM)
        assertEquals(FieldConfig.PRO.copy(hashSpacingM = 12.192), FieldConfig.COLLEGE)
    }

    @Test fun worldConfigDefaultsMatchSpec() {
        val c = WorldConfig()
        assertEquals(8, c.velocityIterations)
        assertEquals(3, c.relaxIterations)
        assertEquals(4, c.maxTicksPerFrame)
        assertEquals(30.0, c.contactHertz)
        assertEquals(10.0, c.contactDampingRatio)
        assertEquals(3.0, c.maxPushoutSpeed)
        assertEquals(0.5, c.sleepTimeS)
        assertEquals(0.05, c.sleepSpeed)
        assertEquals(9.81, c.gravity)
        assertEquals(1.0 / 60.0, WorldConfig.DT)
    }

    @Test fun worldConfigRejectsNonPositiveCounts() {
        assertFailsWith<IllegalArgumentException> { WorldConfig(maxBodies = 0) }
        assertFailsWith<IllegalArgumentException> { WorldConfig(maxTicksPerFrame = 0) }
        assertFailsWith<IllegalArgumentException> { WorldConfig(velocityIterations = 0) }
    }
}
