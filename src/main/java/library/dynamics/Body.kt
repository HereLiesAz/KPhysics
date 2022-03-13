package library.dynamics

import library.collision.AxisAlignedBoundingBox
import library.geometry.Shape
import library.math.Vec2

/**
 * Class to create a body to add to a world.
 *
 * @param shape Shape to bind to body.
 * @param x     Position x in world space.
 * @param y     Position y in world space.
 */
class Body(var shape: Shape, x: Double, y: Double) {
    var dynamicFriction: Double
    var staticFriction: Double
    var position: Vec2
    var velocity: Vec2
    var force: Vec2
    var angularVelocity: Double
    var torque: Double
    var restitution: Double
    var mass = 0.0
    var invMass = 0.0
    var inertia = 0.0
    var invInertia = 0.0
    var orientation: Double = 0.0
        set(value) {
            field = value
            shape.orientation.set(orientation)
            shape.createAABB()
        }
    lateinit var aabb: AxisAlignedBoundingBox
    var linearDampening: Double
    private var angularDampening: Double
    var affectedByGravity: Boolean
    var particle: Boolean

    init {
        shape.body = this
        position = Vec2(x, y)
        velocity = Vec2(.0, .0)
        force = Vec2(.0, .0)
        angularVelocity = 0.0
        torque = 0.0
        restitution = 0.8
        staticFriction = 0.5
        dynamicFriction = 0.2
        linearDampening = 0.0
        angularDampening = 0.0
        orientation = 0.0
        shape.orientation.set(orientation)
        shape.calcMass(1.0)
        shape.createAABB()
        particle = false
        affectedByGravity = true
    }

    /**
     * Applies force ot body.
     *
     * @param force        Force vector to apply.
     * @param contactPoint The point to apply the force to relative to the body in object space.
     */
    fun applyForce(force: Vec2, contactPoint: Vec2) {
        this.force.add(force)
        torque += contactPoint.cross(force)
    }

    /**
     * Apply force to the center of mass.
     *
     * @param force Force vector to apply.
     */
    fun applyForce(force: Vec2) {
        this.force.add(force)
    }

    /**
     * Applies impulse to a point relative to the body's center of mass.
     *
     * @param impulse      Magnitude of impulse vector.
     * @param contactPoint The point to apply the force to relative to the body in object space.
     */
    fun applyLinearImpulse(impulse: Vec2, contactPoint: Vec2) {
        velocity.add(impulse.scalar(invMass))
        angularVelocity += invInertia * contactPoint.cross(impulse)
    }

    /**
     * Applies impulse to body's center of mass.
     *
     * @param impulse Magnitude of impulse vector.
     */
    fun applyLinearImpulse(impulse: Vec2) {
        velocity.add(impulse.scalar(invMass))
    }

    /**
     * Sets the density of the body's mass.
     *
     * @param density double value of desired density.
     */
    fun setDensity(density: Double) {
        if (density > 0.0) {
            shape.calcMass(density)
        } else {
            setStatic()
        }
    }

    /**
     * Sets all mass and inertia variables to zero. Object cannot be moved.
     */
    private fun setStatic() {
        mass = 0.0
        invMass = 0.0
        inertia = 0.0
        invInertia = 0.0
    }
}