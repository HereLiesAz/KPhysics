package library.collision

import library.dynamics.Body
import library.math.Vectors2D

/**
 * Axis aligned bounding box volume class. Allows the creation of bounding volumes to make broad phase collision check possible and easy to do.
 */
class AABB {
    /**
     * Getter for min variable for lower bound vertex.
     *
     * @return AABB min
     */
    /**
     * Lower left vertex of bounding box.
     */
    var min: Vectors2D
        private set
    /**
     * Getter for max variable for upper bound vertex.
     *
     * @return AABB max
     */
    /**
     * Top right vertex of bounding box.
     */
    var max: Vectors2D
        private set

    /**
     * Constructor to generate an AABB given a minimum and maximum bound in the form of two vectors.
     *
     * @param min Lower bound of AABB vertex.
     * @param max Higher bound of AABB vertex.
     */
    constructor(min: Vectors2D, max: Vectors2D) {
        this.min = min.copy()
        this.max = max.copy()
    }

    /**
     * Default constructor generating an AABB with (0,0) upper and lower bounds.
     */
    constructor() {
        min = Vectors2D()
        max = Vectors2D()
    }

    /**
     * Sets the current objects bounds equal to that of the passed AABB argument.
     *
     * @param aabb An AABB bounding box.
     */
    fun set(aabb: AABB) {
        val v = aabb.min
        min.x = v.x
        min.y = v.y
        val v1 = aabb.max
        max.x = v1.x
        max.y = v1.y
    }

    /**
     * Method to check if an AABB is valid.
     * Makes sure the bounding volume is not; a point, has order of vertex's backwards and valid values have been used for the bounds.
     *
     * @return boolean value of the validity of the AABB.
     */
    val isValid: Boolean
        get() {
            if (max.x - min.x < 0) {
                return false
            }
            return if (max.y - min.y < 0) {
                false
            } else min.isValid && max.isValid
        }

    /**
     * Method to check if a point resides inside an AABB in object space.
     *
     * @param point A point to check if its inside the AABB's object space. Point needs to also be in object space.
     * @return Boolean value whether or not the point lies inside the AABB bounds.
     */
    fun AABBOverLap(point: Vectors2D): Boolean {
        val x = point.x
        val y = point.y
        return x <= max.x && x >= min.x && y >= max.y && y <= min.y
    }

    /**
     * Method to add offset to the AABB's bounds. Can be useful to convert from object to world space .
     *
     * @param offset A vector to apply to the min and max vectors to translate the bounds and therefore AABB to desired position.
     */
    fun addOffset(offset: Vectors2D?) {
        min.add(offset)
        max.add(offset)
    }

    override fun toString(): String {
        return "AABB[$min . $max]"
    }

    /**
     * Copy method to return a new AABB that's the same as the current object.
     *
     * @return New AABB that's the same as the current object.
     */
    fun copy(): AABB {
        return AABB(min, max)
    }

    companion object {
        /**
         * Checks whether two body's AABB's overlap in world space.
         *
         * @param a First body to evaluate.
         * @param b Second body to evaluate.
         * @return Boolean value of whether the two bodies AABB's overlap in world space.
         */
        @JvmStatic
        fun aabbOverlap(a: Body, b: Body): Boolean {
            val aCopy = a.aabb.copy()
            val bCopy = b.aabb.copy()
            aCopy.addOffset(a.position)
            bCopy.addOffset(b.position)
            return aabbOverlap(aCopy, bCopy)
        }

        /**
         * Method to check if two AABB's overlap. Can be seen as world space.
         *
         * @param a First AABB to evaluate.
         * @param b Second AABB to evaluate.
         * @return Boolean value of whether two bounds of the AABB's overlap.
         */
        @JvmStatic
        fun aabbOverlap(a: AABB, b: AABB): Boolean {
            return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y
        }
    }
}