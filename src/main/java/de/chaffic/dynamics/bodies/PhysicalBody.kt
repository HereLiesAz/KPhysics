package de.chaffic.dynamics.bodies

import de.chaffic.geometry.Shape
import de.chaffic.geometry.interfaces.Translatable
import de.chaffic.math.Vec2

abstract class PhysicalBody(x: Double, y: Double, var collisionBody: Shape? = null /*replace with collisionBody*/) :
    Translatable {
    override var position = Vec2(x, y)
    var velocity = Vec2()
    var force = Vec2()
    var angularVelocity = .0
    var torque = .0
    var restitution = .8
    var density = 1.0
        /**
         * Sets the density and calculates the mass depending on it.
         *
         * @param value the new value for density.
         */
        set(value) { //TODO not autoCalc
            field = value
            if (density == .0) {
                setStatic()
            } else if (collisionBody != null) {
                collisionBody!!.calcMass(field)
            }
        }
    var mass = .0
    var invMass = .0
    var inertia = .0
    var invInertia = .0
    var angularDampening = .0
    var linearDampening = .0
    var affectedByGravity = true
    var particle = false

    init {
        //collisionBody?.body = this
        //density = density
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
     * Sets all mass and inertia variables to zero. Object cannot be moved.
     */
    private fun setStatic() {
        mass = 0.0
        invMass = 0.0
        inertia = 0.0
        invInertia = 0.0
    }
}