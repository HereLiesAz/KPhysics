package de.chaffic.geometry

import de.chaffic.collision.bodies.CollisionBodyInterface
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2

/**
 * Abstract class presenting a geometric shape.
 */
abstract class Shape {
    lateinit var body: CollisionBodyInterface
    var orientation: Mat2 = Mat2()

    /**
     * Calculates the mass of a shape.
     *
     * @param density The desired density to factor into the calculation.
     */
    abstract fun calcMass(density: Double)

    /**
     * Generates an AABB for the shape.
     */
    abstract fun createAABB()

    abstract fun isPointInside(startPoint: Vec2): Boolean
}