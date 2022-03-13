package library.math

/**
 * 2D Vectors class
 */
class Vectors2D {
    var x: Double
    var y: Double

    /**
     * Default constructor - x/y initialised to zero.
     */
    constructor() {
        x = 0.0
        y = 0.0
    }

    /**
     * Constructor.
     *
     * @param x Sets x value.
     * @param y Sets y value.
     */
    constructor(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    /**
     * Copy constructor.
     *
     * @param vector Vector to copy.
     */
    constructor(vector: Vectors2D?) {
        x = vector!!.x
        y = vector.y
    }

    /**
     * Constructs a normalised direction vector.
     *
     * @param direction Direction in radians.
     */
    constructor(direction: Double) {
        x = Math.cos(direction)
        y = Math.sin(direction)
    }

    /**
     * Sets a vector to equal an x/y value and returns this.
     *
     * @param x x value.
     * @param y y value.
     * @return The current instance vector.
     */
    operator fun set(x: Double, y: Double): Vectors2D {
        this.x = x
        this.y = y
        return this
    }

    /**
     * Sets a vector to another vector and returns this.
     *
     * @param v1 Vector to set x/y values to.
     * @return The current instance vector.
     */
    fun set(v1: Vectors2D?): Vectors2D {
        x = v1!!.x
        y = v1.y
        return this
    }

    /**
     * Copy method to return a new copy of the current instance vector.
     *
     * @return A new Vectors2D object.
     */
    fun copy(): Vectors2D {
        return Vectors2D(x, y)
    }

    /**
     * Negates the current instance vector and return this.
     *
     * @return Return the negative form of the instance vector.
     */
    fun negative(): Vectors2D {
        x = -x
        y = -y
        return this
    }

    /**
     * Negates the current instance vector and return this.
     *
     * @return Returns a new negative vector of the current instance vector.
     */
    fun negativeVec(): Vectors2D {
        return Vectors2D(-x, -y)
    }

    /**
     * Adds a vector to the current instance and return this.
     *
     * @param v Vector to add.
     * @return Returns the current instance vector.
     */
    fun add(v: Vectors2D?): Vectors2D {
        x = x + v!!.x
        y = y + v.y
        return this
    }

    /**
     * Adds a vector and the current instance vector together and returns a new vector of them added together.
     *
     * @param v Vector to add.
     * @return Returns a new Vectors2D of the sum of the addition of the two vectors.
     */
    fun addi(v: Vectors2D?): Vectors2D {
        return Vectors2D(x + v!!.x, y + v.y)
    }

    /**
     * Generates a normal of a vector. Normal facing to the right clock wise 90 degrees.
     *
     * @return A normal of the current instance vector.
     */
    fun normal(): Vectors2D {
        return Vectors2D(-y, x)
    }

    /**
     * Normalizes the current instance vector to length 1 and returns this.
     *
     * @return Returns the normalized version of the current instance vector.
     */
    fun normalize(): Vectors2D {
        var d = Math.sqrt(x * x + y * y)
        if (d == 0.0) {
            d = 1.0
        }
        x /= d
        y /= d
        return this
    }

    /**
     * Finds the normalised version of a vector and returns a new vector of it.
     *
     * @return A normalized vector of the current instance vector.
     */
    val normalized: Vectors2D
        get() {
            var d = Math.sqrt(x * x + y * y)
            if (d == 0.0) {
                d = 1.0
            }
            return Vectors2D(x / d, y / d)
        }

    /**
     * Finds the distance between two vectors.
     *
     * @param v Vector to find distance from.
     * @return Returns distance from vector v to the current instance vector.
     */
    fun distance(v: Vectors2D?): Double {
        val dx = x - v!!.x
        val dy = y - v.y
        return StrictMath.sqrt(dx * dx + dy * dy)
    }

    /**
     * Subtract a vector from the current instance vector.
     *
     * @param v1 Vector to subtract.
     * @return Returns a new Vectors2D with the subtracted vector applied
     */
    fun subtract(v1: Vectors2D?): Vectors2D {
        return Vectors2D(x - v1!!.x, y - v1.y)
    }

    /**
     * Finds cross product between two vectors.
     *
     * @param v1 Other vector to apply cross product to
     * @return double
     */
    fun crossProduct(v1: Vectors2D?): Double {
        return x * v1!!.y - y * v1.x
    }

    fun crossProduct(a: Double): Vectors2D {
        return normal().scalar(a)
    }

    fun scalar(a: Double): Vectors2D {
        return Vectors2D(x * a, y * a)
    }

    /**
     * Finds dotproduct between two vectors.
     *
     * @param v1 Other vector to apply dotproduct to.
     * @return double
     */
    fun dotProduct(v1: Vectors2D?): Double {
        return v1!!.x * x + v1.y * y
    }

    /**
     * Gets the length of instance vector.
     *
     * @return double
     */
    fun length(): Double {
        return Math.sqrt(x * x + y * y)
    }

    /**
     * Checks to see if a vector has valid values set for x and y.
     *
     * @return boolean value whether a vector is valid or not.
     */
    val isValid: Boolean
        get() = !java.lang.Double.isNaN(x) && !java.lang.Double.isInfinite(x) && !java.lang.Double.isNaN(y) && !java.lang.Double.isInfinite(
            y
        )

    /**
     * Checks to see if a vector is set to (0,0).
     *
     * @return boolean value whether the vector is set to (0,0).
     */
    val isZero: Boolean
        get() = Math.abs(x) == 0.0 && Math.abs(y) == 0.0

    override fun toString(): String {
        return x.toString() + " : " + y
    }

    companion object {
        /**
         * Static method for any cross product, same as
         * [.cross]
         *
         * @param s double.
         * @param a Vectors2D.
         * @return Cross product scalar result.
         */
        fun cross(a: Vectors2D, s: Double): Vectors2D {
            return Vectors2D(s * a.y, -s * a.x)
        }

        /**
         * Finds the cross product of a scalar and a vector. Produces a scalar in 2D.
         *
         * @param s double.
         * @param a Vectors2D.
         * @return Cross product scalar result.
         */
        @kotlin.jvm.JvmStatic
        fun cross(s: Double, a: Vectors2D): Vectors2D {
            return Vectors2D(-s * a.y, s * a.x)
        }

        /**
         * Generates an array of length n with zero initialised vectors.
         *
         * @param n Length of array.
         * @return A Vectors2D array of zero initialised vectors.
         */
        @kotlin.jvm.JvmStatic
        fun createArray(n: Int): Array<Vectors2D?> {
            val array = arrayOfNulls<Vectors2D>(n)

            array.forEach {
                if (it != null) {
                    it.x = 0.0
                    it.y = 0.0
                }
            }
            return array
        }
    }
}