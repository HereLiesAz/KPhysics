package library.joints

import library.dynamics.Body
import library.math.Matrix2D
import library.math.Vectors2D
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D

/**
 * Abstract class for joints holding all the common properties of joints.
 */
abstract class Joint protected constructor(
    protected val object1: Body,
    jointLength: Double,
    jointConstant: Double,
    dampening: Double,
    canGoSlack: Boolean,
    offset1: Vectors2D?
) {
    protected val naturalLength: Double
    protected val springConstant: Double
    protected val dampeningConstant: Double
    protected val canGoSlack: Boolean
    protected val offset1: Vectors2D?
    protected var object1AttachmentPoint: Vectors2D?

    /**
     * Default constructor
     *
     * @param b1            A body the joint is attached to
     * @param jointLength   The desired distance of the joint between two points/bodies
     * @param jointConstant The strength of the joint
     * @param dampening     The dampening constant to use for the joints forces
     * @param canGoSlack    Boolean whether the joint can go slack or not
     * @param offset1       Offset to be applied to the location of the joint relative to b1's object space.
     */
    init {
        val u = Matrix2D(object1.orientation)
        object1AttachmentPoint = object1.position.addi(u.mul(offset1, Vectors2D()))
        naturalLength = jointLength
        springConstant = jointConstant
        dampeningConstant = dampening
        this.canGoSlack = canGoSlack
        this.offset1 = offset1
    }

    /**
     * Abstract method to apply tension to the joint
     */
    abstract fun applyTension()

    /**
     * Abstract method to calculate tension between the joint
     *
     * @return double value of the tension force between two points/bodies
     */
    abstract fun calculateTension(): Double

    /**
     * Determines the rate of change between two objects/points.
     * @return double value of the rate of change
     */
    abstract fun rateOfChangeOfExtension(): Double

    /**
     * Abstract draw method using graphics2D from java.swing for debug drawer.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    abstract fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera)
}