package library.geometry

import library.dynamics.Body
import library.math.Matrix2D
import library.math.Vectors2D
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D
import java.awt.geom.Line2D
import java.awt.geom.Path2D

/**
 * Abstract class presenting a geometric shape.
 */
abstract class Shapes internal constructor() {
    lateinit var body: Body
    var orient: Matrix2D

    /**
     * Default constructor
     */
    init {
        orient = Matrix2D()
    }

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

    /**
     * Debug draw method for shape.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    abstract fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera)

    /**
     * Debug draw method for AABB.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun drawAABB(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        g.color = paintSettings.aabb
        val polyBB: Path2D = Path2D.Double()
        val min = camera.convertToScreen(body.aabb.min.addi(body.position))
        val max = camera.convertToScreen(body.aabb.max.addi(body.position))
        polyBB.moveTo(min.x, min.y)
        polyBB.lineTo(min.x, max.y)
        polyBB.lineTo(max.x, max.y)
        polyBB.lineTo(max.x, min.y)
        polyBB.closePath()
        g.draw(polyBB)
    }

    /**
     * Debug draw method for center of mass.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun drawCOMS(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        g.color = paintSettings.centreOfMass
        val centre = body.position
        val line = Vectors2D(paintSettings.COM_RADIUS.toDouble(), .0)
        orient.mul(line)
        var beginningOfLine = camera.convertToScreen(centre.addi(line))
        var endOfLine = camera.convertToScreen(centre.subtract(line))
        val lin1: Line2D = Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y)
        g.draw(lin1)
        beginningOfLine = camera.convertToScreen(centre.addi(line.normal()))
        endOfLine = camera.convertToScreen(centre.subtract(line.normal()))
        val lin2: Line2D = Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y)
        g.draw(lin2)
    }
}