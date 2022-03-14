package de.chaffic.geometry

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.math.Vec2

/**
 * Circle class to create a circle object.
 */
class Circle
/**
 * Constructor for a circle.
 *
 * @param radius Desired radius of the circle.
 */(var radius: Double) : Shape() {
    /**
     * Calculates the mass of a circle.
     *
     * @param density The desired density to factor into the calculation.
     */
    override fun calcMass(density: Double) {
        val physicalBody = this.body
        if(physicalBody !is PhysicalBodyInterface) return
        physicalBody.mass = StrictMath.PI * radius * radius * density
        physicalBody.invMass = if (physicalBody.mass != 0.0) 1.0f / physicalBody.mass else 0.0
        physicalBody.inertia = physicalBody.mass * radius * radius
        physicalBody.invInertia = if (physicalBody.inertia != 0.0) 1.0f / physicalBody.inertia else 0.0
    }

    /**
     * Generates an AABB and binds it to the body.
     */
    override fun createAABB() {
        this.body.aabb = AxisAlignedBoundingBox(Vec2(-radius, -radius), Vec2(radius, radius))
    }

    /**
     * Method to check if point is inside a body in world space.
     *
     * @param startPoint Vector point to check if its inside the first body.
     * @return boolean value whether the point is inside the first body.
     */
    override fun isPointInside(startPoint: Vec2): Boolean {
        val d = this.body.position.minus(startPoint)
        return d.length() <= radius
    }
}