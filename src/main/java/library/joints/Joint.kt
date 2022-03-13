package library.joints

import demo.Camera
import demo.ColourSettings
import library.dynamics.Body
import library.math.Mat2
import library.math.Vec2
import java.awt.Graphics2D

/**
 * Abstract class for joints holding all the common properties of joints.
 *
 * @param body            A body the joint is attached to
 * @param naturalLength   The desired distance of the joint between two points/bodies
 * @param springConstant The strength of the joint
 * @param dampeningConstant     The dampening constant to use for the joints forces
 * @param canGoSlack    Boolean whether the joint can go slack or not
 * @param offset       Offset to be applied to the location of the joint relative to b1's object space.
 */
abstract class Joint protected constructor(
    protected val body: Body,
    protected val naturalLength: Double,
    protected val springConstant: Double,
    protected val dampeningConstant: Double,
    protected val canGoSlack: Boolean,
    protected val offset: Vec2
) {
    protected var object1AttachmentPoint: Vec2

    init {
        val u = Mat2(body.orientation)
        object1AttachmentPoint = body.position.plus(u.mul(offset, Vec2()))
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