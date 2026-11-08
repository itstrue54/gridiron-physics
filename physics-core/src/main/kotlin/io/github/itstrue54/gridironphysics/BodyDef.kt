package io.github.itstrue54.gridironphysics

/**
 * Parameters for [World.createBody]. Mutable so the game can keep one and reuse it.
 * `mass` is ignored for Static and Kinematic bodies.
 */
class BodyDef(
    var type: BodyType = BodyType.Dynamic,
    var x: Double = 0.0,
    var y: Double = 0.0,
    var angle: Double = 0.0,
    var vx: Double = 0.0,
    var vy: Double = 0.0,
    var angularVelocity: Double = 0.0,
    var mass: Double = 100.0,
    var linearDamping: Double = 0.0,
    var angularDamping: Double = 0.0,
    var shape: ShapeDef = ShapeDef.Circle(0.34),
    var userData: Int = 0,
    var bullet: Boolean = false,
) {
    /** Copies every field of [other] into this def without allocating. Returns this. */
    fun copyFrom(other: BodyDef): BodyDef {
        type = other.type
        x = other.x; y = other.y; angle = other.angle
        vx = other.vx; vy = other.vy; angularVelocity = other.angularVelocity
        mass = other.mass
        linearDamping = other.linearDamping; angularDamping = other.angularDamping
        shape = other.shape // ShapeDef is immutable, so sharing it is safe
        userData = other.userData; bullet = other.bullet
        return this
    }

    internal fun validate() {
        require(x.isFinite() && y.isFinite() && angle.isFinite()) { "BodyDef position/angle must be finite: ($x, $y, $angle)" }
        require(vx.isFinite() && vy.isFinite() && angularVelocity.isFinite()) { "BodyDef velocity must be finite: ($vx, $vy, $angularVelocity)" }
        require(linearDamping.isFinite() && linearDamping >= 0.0) { "linearDamping must be finite and >= 0: $linearDamping" }
        require(angularDamping.isFinite() && angularDamping >= 0.0) { "angularDamping must be finite and >= 0: $angularDamping" }
        if (type == BodyType.Dynamic) require(mass.isFinite() && mass > 0.0) { "Dynamic body mass must be finite and > 0: $mass" }
        when (val s = shape) {
            is ShapeDef.Circle -> require(s.radius.isFinite() && s.radius > 0.0) { "Circle radius must be finite and > 0: ${s.radius}" }
        }
    }
}
