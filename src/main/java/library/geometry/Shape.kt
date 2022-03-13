package library.geometry

import demo.Camera
import demo.ColourSettings
import library.dynamics.Body
import library.math.Mat2
import library.math.Vec2
import java.awt.Graphics2D
import java.awt.geom.Line2D
import java.awt.geom.Path2D

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
        val min = camera.convertToScreen(body.aabb.min.plus(body.position))
        val max = camera.convertToScreen(body.aabb.max.plus(body.position))
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
        val line = Vec2(paintSettings.COM_RADIUS.toDouble(), .0)
        orientation.mul(line)
        var beginningOfLine = camera.convertToScreen(centre.plus(line))
        var endOfLine = camera.convertToScreen(centre.minus(line))
        val lin1: Line2D = Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y)
        g.draw(lin1)
        beginningOfLine = camera.convertToScreen(centre.plus(line.normal()))
        endOfLine = camera.convertToScreen(centre.minus(line.normal()))
        val lin2: Line2D = Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y)
        g.draw(lin2)
    }
}