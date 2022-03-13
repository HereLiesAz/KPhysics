package library.geometry

import library.collision.AxisAlignedBoundingBox
import library.math.Vec2

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
        body.mass = StrictMath.PI * radius * radius * density
        body.invMass = if (body.mass != 0.0) 1.0f / body.mass else 0.0
        body.inertia = body.mass * radius * radius
        body.invInertia = if (body.inertia != 0.0) 1.0f / body.inertia else 0.0
    }

    /**
     * Generates an AABB and binds it to the body.
     */
    override fun createAABB() {
        body.aabb = AxisAlignedBoundingBox(Vec2(-radius, -radius), Vec2(radius, radius))
    }

    /**
     * Method to check if point is inside a body in world space.
     *
     * @param startPoint Vector point to check if its inside the first body.
     * @return boolean value whether the point is inside the first body.
     */
    override fun isPointInside(startPoint: Vec2): Boolean {
        val d = body.position.minus(startPoint)
        return d.length() <= radius
    }
}