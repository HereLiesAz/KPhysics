package library.dynamics

import library.collision.AABB
import library.geometry.Shapes
import library.math.Vectors2D

/**
 * Class to create a body to add to a world.
 */
class Body(var shape: Shapes, x: Double, y: Double) {
    var dynamicFriction: Double
    var staticFriction: Double
    var position: Vectors2D
    var velocity: Vectors2D
    var force: Vectors2D
    var angularVelocity: Double
    var torque: Double
    var restitution: Double
    var mass = 0.0
    var invMass = 0.0
    var I = 0.0
    var invI = 0.0
    var orientation: Double = 0.0
        set(value) {
            field = value
            shape.orient.set(orientation)
            shape.createAABB()
        }
    lateinit var aabb: AABB
    var linearDampening: Double
    var angularDampening: Double
    var affectedByGravity: Boolean
    var particle: Boolean

    /**
     * Constructor for body.
     *
     * @param shape Shape to bind to body.
     * @param x     Position x in world space.
     * @param y     Position y in world space.
     */
    init {
        shape.body = this
        position = Vectors2D(x, y)
        velocity = Vectors2D(.0, .0)
        force = Vectors2D(.0, .0)
        angularVelocity = 0.0
        torque = 0.0
        restitution = 0.8
        staticFriction = 0.5
        dynamicFriction = 0.2
        linearDampening = 0.0
        angularDampening = 0.0
        orientation = 0.0
        shape.orient.set(orientation)
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
    fun applyForce(force: Vectors2D?, contactPoint: Vectors2D) {
        this.force.add(force)
        torque += contactPoint.crossProduct(force)
    }

    /**
     * Apply force to the center of mass.
     *
     * @param force Force vector to apply.
     */
    fun applyForceToCentre(force: Vectors2D?) {
        this.force.add(force)
    }

    /**
     * Applies impulse to a point relative to the body's center of mass.
     *
     * @param impulse      Magnitude of impulse vector.
     * @param contactPoint The point to apply the force to relative to the body in object space.
     */
    fun applyLinearImpulse(impulse: Vectors2D?, contactPoint: Vectors2D?) {
        velocity.add(impulse!!.scalar(invMass))
        angularVelocity += invI * contactPoint!!.crossProduct(impulse)
    }

    /**
     * Applies impulse to body's center of mass.
     *
     * @param impulse Magnitude of impulse vector.
     */
    fun applyLinearImpulseToCentre(impulse: Vectors2D?) {
        velocity.add(impulse!!.scalar(invMass))
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
        I = 0.0
        invI = 0.0
    }
}