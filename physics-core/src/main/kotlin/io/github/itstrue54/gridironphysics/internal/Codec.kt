package io.github.itstrue54.gridironphysics.internal

import io.github.itstrue54.gridironphysics.AthleteParams
import io.github.itstrue54.gridironphysics.BallConfig
import io.github.itstrue54.gridironphysics.FieldConfig
import io.github.itstrue54.gridironphysics.MaterialTable
import io.github.itstrue54.gridironphysics.ShapeDef
import io.github.itstrue54.gridironphysics.SurfaceConfig
import io.github.itstrue54.gridironphysics.WorldConfig

/** Binary encodings of the engine's value types, shared by snapshots and replays. Field order is the format. */
internal object Codec {
    private const val CIRCLE = 0
    private const val CAPSULE = 1
    private const val POLYGON = 2
    private const val SEGMENT = 3

    fun shape(s: ByteSink, shape: ShapeDef) {
        when (shape) {
            is ShapeDef.Circle -> { s.u8(CIRCLE); s.f64(shape.radius) }
            is ShapeDef.Capsule -> { s.u8(CAPSULE); s.f64(shape.halfLength); s.f64(shape.radius) }
            is ShapeDef.Polygon -> {
                s.u8(POLYGON); s.u8(shape.count)
                // Stored counter-clockwise, so rebuilding from these vertices gives the same polygon bit for bit.
                for (i in 0 until 2 * shape.count) s.f64(shape.vertex(i))
            }
            is ShapeDef.Segment -> { s.u8(SEGMENT); s.f64(shape.x1); s.f64(shape.y1); s.f64(shape.x2); s.f64(shape.y2) }
        }
        s.bool(shape.sensor)
    }

    fun shape(r: ByteSource): ShapeDef {
        val kind = r.u8()
        return when (kind) {
            CIRCLE -> { val rad = r.f64(); ShapeDef.Circle(rad, r.bool()) }
            CAPSULE -> { val h = r.f64(); val rad = r.f64(); ShapeDef.Capsule(h, rad, r.bool()) }
            POLYGON -> {
                val n = r.u8()
                if (n !in 3..ShapeDef.MAX_VERTICES) throw FormatException("bad polygon vertex count $n")
                val v = DoubleArray(2 * n) { r.f64() }
                ShapeDef.Polygon(v, r.bool())
            }
            SEGMENT -> { val x1 = r.f64(); val y1 = r.f64(); val x2 = r.f64(); val y2 = r.f64(); ShapeDef.Segment(x1, y1, x2, y2, r.bool()) }
            else -> throw FormatException("bad shape kind $kind")
        }
    }

    fun athlete(s: ByteSink, p: AthleteParams) {
        s.f64(p.maxSpeed); s.f64(p.maxAccel); s.f64(p.maxDecel)
        s.f64(p.turnRateAtRest); s.f64(p.turnRateAtMax); s.f64(p.traction); s.f64(p.balance)
    }

    fun athlete(r: ByteSource): AthleteParams = AthleteParams(
        maxSpeed = r.f64(), maxAccel = r.f64(), maxDecel = r.f64(),
        turnRateAtRest = r.f64(), turnRateAtMax = r.f64(), traction = r.f64(), balance = r.f64(),
    )

    fun surface(s: ByteSink, v: SurfaceConfig) {
        s.str(v.name); s.f64(v.tractionMultiplier); s.f64(v.ballBounceMultiplier)
    }

    /** The preset instance when the values match one, so a restored surface still compares and prints as before. */
    fun surface(r: ByteSource): SurfaceConfig {
        val name = r.str(); val t = r.f64(); val b = r.f64()
        for (p in PRESETS) {
            if (p.name == name && p.tractionMultiplier.toRawBits() == t.toRawBits() && p.ballBounceMultiplier.toRawBits() == b.toRawBits()) return p
        }
        return SurfaceConfig(name, t, b)
    }

    private val PRESETS = arrayOf(SurfaceConfig.ARTIFICIAL_TURF, SurfaceConfig.NATURAL_GRASS, SurfaceConfig.WET, SurfaceConfig.FROZEN)

    fun field(s: ByteSink, f: FieldConfig) {
        s.f64(f.lengthM); s.f64(f.widthM); s.f64(f.endZoneDepthM); s.f64(f.hashSpacingM)
        s.f64(f.crossbarHeightM); s.f64(f.uprightSpacingM); s.f64(f.uprightHeightAboveCrossbarM); s.f64(f.runoffM)
        s.f64(f.uprightRadiusM); s.f64(f.goalPostOffsetM); s.f64(f.goalPostBaseHalfM)
    }

    fun field(r: ByteSource): FieldConfig = FieldConfig(
        lengthM = r.f64(), widthM = r.f64(), endZoneDepthM = r.f64(), hashSpacingM = r.f64(),
        crossbarHeightM = r.f64(), uprightSpacingM = r.f64(), uprightHeightAboveCrossbarM = r.f64(), runoffM = r.f64(),
        uprightRadiusM = r.f64(), goalPostOffsetM = r.f64(), goalPostBaseHalfM = r.f64(),
    )

    fun ball(s: ByteSink, b: BallConfig) {
        s.f64(b.mass); s.f64(b.radius); s.f64(b.crossSection); s.f64(b.airDensity)
        s.f64(b.cdSpiral); s.f64(b.cdWobble); s.f64(b.cdEndOverEnd); s.bool(b.drag)
        s.f64(b.bounceYawMax); s.f64(b.minBounceSpeed); s.f64(b.rollFriction); s.f64(b.catchMinHeight); s.f64(b.carryHeight)
    }

    fun ball(r: ByteSource): BallConfig = BallConfig(
        mass = r.f64(), radius = r.f64(), crossSection = r.f64(), airDensity = r.f64(),
        cdSpiral = r.f64(), cdWobble = r.f64(), cdEndOverEnd = r.f64(), drag = r.bool(),
        bounceYawMax = r.f64(), minBounceSpeed = r.f64(), rollFriction = r.f64(), catchMinHeight = r.f64(), carryHeight = r.f64(),
    )

    fun materials(s: ByteSink, m: MaterialTable) {
        val n = MaterialTable.SIZE
        s.u8(n)
        for (a in 0 until n) for (b in 0 until n) { s.f64(m.friction(a, b)); s.f64(m.restitution(a, b)) }
    }

    fun materials(r: ByteSource): MaterialTable {
        val n = r.u8()
        require(n == MaterialTable.SIZE) { "material table size $n, this build uses ${MaterialTable.SIZE}" }
        val mu = DoubleArray(n * n); val e = DoubleArray(n * n)
        for (i in 0 until n * n) { mu[i] = r.f64(); e[i] = r.f64() }
        return MaterialTable.fromRaw(mu, e)
    }

    fun config(s: ByteSink, c: WorldConfig) {
        s.i32(c.maxBodies); s.i32(c.maxTicksPerFrame); s.i32(c.velocityIterations); s.i32(c.relaxIterations)
        s.f64(c.contactHertz); s.f64(c.contactDampingRatio); s.f64(c.maxPushoutSpeed)
        s.f64(c.sleepTimeS); s.f64(c.sleepSpeed); s.f64(c.gravity); s.bool(c.hashEveryTick)
        s.f64(c.restitutionThreshold); materials(s, c.materials); s.bool(c.runoffWalls); s.i32(c.maxContacts)
        s.f64(c.slideFriction); s.i32(c.eventCapacity); s.f64(c.tripMinSpeed); s.i32(c.maxJoints)
        s.f64(c.jointHertz); s.f64(c.jointDampingRatio); ball(s, c.ball); s.bool(c.goalPosts)
    }

    fun config(r: ByteSource): WorldConfig = WorldConfig(
        maxBodies = r.i32(), maxTicksPerFrame = r.i32(), velocityIterations = r.i32(), relaxIterations = r.i32(),
        contactHertz = r.f64(), contactDampingRatio = r.f64(), maxPushoutSpeed = r.f64(),
        sleepTimeS = r.f64(), sleepSpeed = r.f64(), gravity = r.f64(), hashEveryTick = r.bool(),
        restitutionThreshold = r.f64(), materials = materials(r), runoffWalls = r.bool(), maxContacts = r.i32(),
        slideFriction = r.f64(), eventCapacity = r.i32(), tripMinSpeed = r.f64(), maxJoints = r.i32(),
        jointHertz = r.f64(), jointDampingRatio = r.f64(), ball = ball(r), goalPosts = r.bool(),
    )
}
