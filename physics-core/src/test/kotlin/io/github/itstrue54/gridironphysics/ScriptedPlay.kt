package io.github.itstrue54.gridironphysics

import io.github.itstrue54.gridironphysics.math.Vec2
import io.github.itstrue54.gridironphysics.math.Vec3

/**
 * A 600-tick play that uses every World command and every piece of state a snapshot must carry: athletes with
 * intents and stances, all four joint kinds (one breaks, one is destroyed), a pass with catch windows and wind, a
 * catch, a fumble that bounces (consuming the rng) and rolls loose, a pickup, a kick off the upright, a body destroyed
 * mid-play and one created mid-play, surface and attack-direction changes, a polygon prop, a kinematic official and
 * a sleeping pile. [commands] runs before each step; call [step] to run the whole tick.
 */
class ScriptedPlay(seed: Long = 42L, val config: WorldConfig = WorldConfig()) {
    val field = FieldConfig.PRO
    val world = World(config, field, seed)
    private val w = world
    private val intent = LocomotionIntent()
    private val v2 = Vec2()
    private val from = Vec3()
    private val vel = Vec3()
    private val tgt = Vec3()
    private val def = BodyDef()

    private var qb = BodyId(-1)
    private var wr = BodyId(-1)
    private var lb = BodyId(-1)
    private var dl = BodyId(-1)
    private var ol = BodyId(-1)
    private var cb = BodyId(-1)
    private var safety = BodyId(-1)
    private var late = BodyId(-1)
    private var grapple = JointId(-1)
    private var block = JointId(-1)
    private var weld = JointId(-1)

    /** Bodies created before the first step. */
    fun setup(): ScriptedPlay {
        w.setSurface(SurfaceConfig.NATURAL_GRASS)
        w.setWind(-2.0, 1.5)
        fun athlete(x: Double, y: Double, mass: Double, cat: Int, p: AthleteParams): BodyId {
            val id = w.createBody(def.apply {
                type = BodyType.Dynamic; this.x = x; this.y = y; angle = if (cat == Category.AWAY) 3.14159 else 0.0
                vx = 0.0; vy = 0.0; angularVelocity = 0.0; this.mass = mass; linearDamping = 0.0; angularDamping = 0.0
                shape = ShapeDef.Circle(0.34); userData = cat; bullet = false; categoryBits = cat
                maskBits = Category.ALL; material = Material.PLAYER
            })
            w.setAthlete(id, p)
            return id
        }
        val fast = AthleteParams(maxSpeed = 9.3, maxAccel = 8.0, traction = 1.0)
        val big = AthleteParams(maxSpeed = 6.8, maxAccel = 5.0, turnRateAtRest = 8.0, traction = 0.95)
        qb = athlete(40.0, 24.0, 100.0, Category.HOME, AthleteParams())
        wr = athlete(45.0, 10.0, 88.0, Category.HOME, fast)
        lb = athlete(50.0, 26.0, 112.0, Category.AWAY, AthleteParams(maxSpeed = 8.4))
        ol = athlete(44.0, 30.0, 140.0, Category.HOME, big)
        dl = athlete(45.0, 30.0, 135.0, Category.AWAY, big)
        cb = athlete(55.0, 11.0, 86.0, Category.AWAY, fast)
        safety = athlete(70.0, 20.0, 95.0, Category.AWAY, fast)
        // Props: a kinematic official, a static polygon, a capsule sled, a pile that falls asleep.
        w.createBody(BodyDef(type = BodyType.Kinematic, x = 30.0, y = -1.5, vx = 1.1, shape = ShapeDef.Circle(0.3), categoryBits = Category.OFFICIAL))
        w.createBody(BodyDef(type = BodyType.Static, x = 60.0, y = 40.0, shape = ShapeDef.Polygon(doubleArrayOf(-1.0, -0.5, 1.0, -0.5, 0.0, 0.8)), material = Material.STRUCTURE, categoryBits = Category.STATIC))
        w.createBody(BodyDef(x = 52.0, y = 42.0, mass = 250.0, linearDamping = 2.0, angle = 0.4, shape = ShapeDef.Capsule(0.6, 0.3), material = Material.STRUCTURE, bullet = true))
        for (k in 0 until 3) w.createBody(BodyDef(x = 20.0 + 0.66 * k, y = 40.0, mass = 100.0, linearDamping = 1.0, angularDamping = 1.0))
        w.setReach(wr, 1.2, 2.8)
        w.setReach(cb, 1.1, 2.7)
        w.setBallCarrier(qb)
        w.setIntent(wr, intent.setVelocity(4.0, 8.0))
        w.setIntent(lb, intent.setPoint(42.0, 24.0, 0.9))
        w.setIntent(ol, intent.setVelocity(1.0, 0.0, 0.6))
        w.setIntent(dl, intent.setVelocity(-1.0, 0.0, 0.7))
        w.setIntent(cb, intent.setVelocity(3.0, 6.0, 0.95))
        w.setIntent(qb, intent.setVelocity(-2.0, 0.0, 0.5))
        block = w.createBlock(ol, dl, BlockDef(blockerPushForce = 900.0, defenderPushForce = 850.0, shedThreshold = 4000.0))
        return this
    }

    /** The game's calls for the coming tick (world.tick = t). */
    fun commands() {
        val t = w.tick.toInt()
        when (t) {
            20 -> w.setIntent(wr, intent.setVelocity(9.0, 1.0))
            30 -> {
                // The pass comes down 1.4 m up on a spot 12 m ahead of the receiver, who runs there and waits.
                val r = w.body(wr) // the view is shared: read it before asking for another body
                tgt.set(r.x + 12.0, r.y + 2.0, 1.4)
                val q = w.body(qb)
                from.set(q.x, q.y, 1.9)
                w.launchVelocityFor(from, tgt, 7.5, FlightMode.Spiral, vel)
                w.launchBall(from, vel, FlightMode.Spiral)
                w.setIntent(wr, intent.setPoint(tgt.x, tgt.y))
                w.setIntent(qb, intent.setIdle())
            }
            40 -> w.setIntent(lb, intent.setPoint(60.0, 18.0))
            45 -> w.setAttackDirection(-1)
            50 -> w.setAttackDirection(1)
            60 -> w.deflect(cb, v2.set(20.0, -15.0))
            70 -> w.setWind(1.0, -0.5)
            90 -> weld = w.createWeld(lb, safety, WeldDef(hertz = 3.0, dampingRatio = 0.7))
            100 -> w.destroyJoint(weld)
            110 -> w.setSurface(SurfaceConfig.WET)
            130 -> w.createDistance(qb, ol, DistanceDef(hertz = 2.0))
        }
        // The catch: give the ball to the receiver as soon as it is in his window (the game's call).
        if (w.ball().state == BallState.Airborne && t in 31..200) {
            // The catch rule looks at the ball directly; events pile up in the ring buffer (a snapshot carries them).
            val r = w.body(wr); val b = w.ball()
            val dx = b.x - r.x; val dy = b.y - r.y
            if (b.z in 0.3..2.8 && dx * dx + dy * dy < 1.44) w.setBallCarrier(wr)
        }
        if (t == 220 && w.ballCarrier == wr) {
            grapple = w.createGrapple(cb, wr, GrappleDef(breakForce = 1800.0, dragForce = 300.0, length = 0.8))
        }
        if (t == 260 && w.ballCarrier == wr) {
            // Fumble: the ball pops out forward and bounces (seeded yaw).
            val r = w.body(wr)
            w.launchBall(from.set(r.x, r.y, 1.0), vel.set(r.vx + 3.0, 2.0, 4.0), FlightMode.EndOverEnd)
            w.setStance(wr, Stance.Diving)
        }
        if (t == 275) {
            intent.setIdle().stance = Stance.Down
            w.setIntent(wr, intent)
            intent.stance = null // the script's intent object is reused: keep its stance from leaking into later calls
        }
        if (t == 300) {
            if (w.hasJoint(grapple)) w.destroyJoint(grapple)
            w.setIntent(safety, intent.setPoint(w.ball().x, w.ball().y))
        }
        if (t in 301..420 && w.ballCarrier == null && w.ball().state == BallState.Loose) {
            val s = w.body(safety); val b = w.ball()
            val dx = b.x - s.x; val dy = b.y - s.y
            if (dx * dx + dy * dy < 1.0) {
                w.setBallCarrier(safety)
                w.setIntent(safety, intent.setVelocity(-9.0, 0.0))
            } else if (t % 10 == 0) {
                w.setIntent(safety, intent.setPoint(b.x, b.y))
            }
        }
        if (t == 330) w.setStance(wr, Stance.Upright)
        if (t == 340) {
            late = w.createBody(BodyDef(x = 30.0, y = 20.0, mass = 90.0, shape = ShapeDef.Circle(0.33), categoryBits = Category.HOME))
            w.setAthlete(late, AthleteParams(maxSpeed = 8.0))
            w.setIntent(late, intent.setVelocity(5.0, 0.0))
        }
        if (t == 360) w.destroyBody(dl) // its block goes with it
        if (t == 430) {
            // A field goal from the safety's spot that hits the left upright.
            if (w.ballCarrier != null) w.setBallCarrier(null)
            from.set(field.lengthM - 30.0, field.widthM / 2, 0.0)
            w.launchVelocityFor(from, tgt.set(field.lengthM, field.widthM / 2 + field.uprightSpacingM / 2, 6.0), 26.0, false, FlightMode.EndOverEnd, vel)
            w.launchBall(from, vel, FlightMode.EndOverEnd)
        }
        if (t == 570) {
            // The whistle: the game hands the ball to a player and takes it back, leaving it Dead.
            w.setBallCarrier(qb)
            w.setBallCarrier(null)
        }
        if (t == 500 && w.ball().state != BallState.Dead && w.ball().state != BallState.Carried) w.deflect(w.ballId, v2.set(0.2, 0.1), if (w.ball().state == BallState.Loose) 1.0 else 0.0)
    }

    /** Takes over [o]'s script variables (joint and body ids), for continuing its play in a restored world. */
    fun copyScriptFrom(o: ScriptedPlay): ScriptedPlay {
        qb = o.qb; wr = o.wr; lb = o.lb; dl = o.dl; ol = o.ol; cb = o.cb; safety = o.safety
        late = o.late
        grapple = o.grapple; block = o.block; weld = o.weld
        return this
    }

    fun step() {
        commands()
        w.step()
    }

    /** Runs [ticks] ticks and returns the hash after each. */
    fun run(ticks: Int): LongArray = LongArray(ticks) { step(); w.stateHash() }

    companion object {
        const val TICKS = 600
    }
}
