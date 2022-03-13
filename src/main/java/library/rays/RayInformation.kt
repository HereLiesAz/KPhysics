package library.rays

import library.dynamics.Body
import library.math.Vectors2D

/**
 * Ray information class to store relevant data about rays and any intersection found.
 */
class RayInformation {
    /**
     * Getter for body variable.
     *
     * @return returns b variable of type Body.
     */
    val b: Body

    /**
     * Getter for coord variable.
     *
     * @return returns coord variable of type Vectors2D.
     */
    val coord: Vectors2D

    /**
     * Getter for index variable.
     *
     * @return returns index variable of type int.
     */
    // Poly index is the first index of the line of intersection found
    val index: Int

    /**
     * Constructor to store information about a ray intersection.
     *
     * @param b     Body involved with ray intersection.
     * @param x     x position of intersection.
     * @param y     y position of intersection.
     * @param index Index of shapes side that intersection intersects.
     */
    constructor(b: Body, x: Double, y: Double, index: Int) {
        this.b = b
        coord = Vectors2D(x, y)
        this.index = index
    }

    /**
     * Convenience constructor equivalent to
     * [.RayInformation]
     *
     * @param b     Body involved with ray intersection.
     * @param v     x/y position of intersection.
     * @param index Index of shapes side that intersection intersects.
     */
    constructor(b: Body, v: Vectors2D, index: Int) {
        this.b = b
        coord = v.copy()
        this.index = index
    }
}