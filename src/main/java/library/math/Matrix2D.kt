package library.math

class Matrix2D {
    @kotlin.jvm.JvmField
    var row1 = Vectors2D()

    @kotlin.jvm.JvmField
    var row2 = Vectors2D()

    /**
     * Default constructor matrix [(0,0),(0,0)] by default.
     */
    constructor() {}

    /**
     * Constructs and sets the matrix up to be a rotation matrix that stores the angle specified in the matrix.
     * @param radians The desired angle of the rotation matrix
     */
    constructor(radians: Double) {
        this.set(radians)
    }

    /**
     * Sets the matrix up to be a rotation matrix that stores the angle specified in the matrix.
     * @param radians The desired angle of the rotation matrix
     */
    fun set(radians: Double) {
        val c = StrictMath.cos(radians)
        val s = StrictMath.sin(radians)
        row1.x = c
        row1.y = -s
        row2.x = s
        row2.y = c
    }

    /**
     * Sets current object matrix to be the same as the supplied parameters matrix.
     * @param m Matrix to set current object to
     */
    fun set(m: Matrix2D) {
        row1.x = m.row1.x
        row1.y = m.row1.y
        row2.x = m.row2.x
        row2.y = m.row2.y
    }

    fun transpose(): Matrix2D {
        val mat = Matrix2D()
        mat.row1.x = row1.x
        mat.row1.y = row2.x
        mat.row2.x = row1.y
        mat.row2.y = row2.y
        return mat
    }

    fun mul(v: Vectors2D?): Vectors2D {
        val x = v!!.x
        val y = v.y
        v.x = row1.x * x + row1.y * y
        v.y = row2.x * x + row2.y * y
        return v
    }

    fun mul(v: Vectors2D?, out: Vectors2D): Vectors2D {
        out.x = row1.x * v!!.x + row1.y * v.y
        out.y = row2.x * v.x + row2.y * v.y
        return out
    }

    override fun toString(): String {
        return """${row1.x} : ${row1.y}
${row2.x} : ${row2.y}"""
    }

    companion object {
        fun main(args: Array<String>) {
            val test = Vectors2D(5.0, .0)
            val m = Matrix2D()
            m.set(0.5)
            m.mul(test)
            println(test)
        }
    }
}