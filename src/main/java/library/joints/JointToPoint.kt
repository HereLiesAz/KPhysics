package library.joints

import library.dynamics.Body
import library.math.Mat2
import library.math.Vec2
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D
import java.awt.geom.Line2D

/**
 * Class for a joint between a body and a point in world space.
 */
class JointToPoint
/**
 * Convenience constructor that works like
 * [.JointToPoint]
 *
 * @param pointAttachedTo The point the joint is attached to
 * @param b1            First body the joint is attached to
 * @param jointLength   The desired distance of the joint between two points/bodies
 * @param jointConstant The strength of the joint
 * @param dampening     The dampening constant to use for the joints forces
 * @param canGoSlack    Boolean whether the joint can go slack or not
 * @param offset       Offset to be applied to the location of the joint relative to b1's object space
 */(
    b1: Body,
    private val pointAttachedTo: Vec2,
    jointLength: Double,
    jointConstant: Double,
    dampening: Double,
    canGoSlack: Boolean,
    offset: Vec2
) : Joint(b1, jointLength, jointConstant, dampening, canGoSlack, offset) {

    /**
     * Applies tension to the body attached to the joint.
     */
    override fun applyTension() {
        val mat1 = Mat2(body.orientation)
        object1AttachmentPoint = body.position.plus(mat1.mul(offset, Vec2()))
        val tension = calculateTension()
        val distance = pointAttachedTo.minus(object1AttachmentPoint)
        distance.normalize()
        val impulse = distance.scalar(tension)
        body.applyLinearImpulse(impulse, object1AttachmentPoint.minus(body.position))
    }

    /**
     * Calculates tension between the two attachment points of the joints body and point.
     *
     * @return double value of the tension force between the point and attached bodies point
     */
    override fun calculateTension(): Double {
        val distance = object1AttachmentPoint.minus(pointAttachedTo).length()
        if (distance < naturalLength && canGoSlack) {
            return .0
        }
        val extensionRatio = distance - naturalLength
        val tensionDueToHooksLaw = extensionRatio * springConstant
        val tensionDueToMotionDamping = dampeningConstant * rateOfChangeOfExtension()
        return tensionDueToHooksLaw + tensionDueToMotionDamping
    }

    /**
     * Determines the rate of change between the attached point and body.
     *
     * @return double value of the rate of change
     */
    override fun rateOfChangeOfExtension(): Double {
        val distance = pointAttachedTo.minus(object1AttachmentPoint)
        distance.normalize()
        val relativeVelocity = body.velocity.copyNegative()
            .minus(object1AttachmentPoint.minus(body.position).cross(body.angularVelocity))
        return relativeVelocity.dot(distance)
    }

    /**
     * Implementation of the draw method.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    override fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        g.color = paintSettings.joints
        val obj1Pos = camera.convertToScreen(object1AttachmentPoint)
        val obj2Pos = camera.convertToScreen(pointAttachedTo)
        g.draw(Line2D.Double(obj1Pos.x, obj1Pos.y, obj2Pos.x, obj2Pos.y))
    }
}