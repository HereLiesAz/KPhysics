package library.geometry

import library.dynamics.Body
import library.math.Mat2

/**
 * Abstract class presenting a geometric shape.
 */
abstract class Shape internal constructor() {
    lateinit var body: Body
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
}