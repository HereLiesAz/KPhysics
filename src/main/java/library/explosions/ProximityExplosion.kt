package library.explosions

import library.dynamics.Body
import library.math.Vectors2D
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D
import java.awt.geom.Ellipse2D
import java.awt.geom.Line2D

/**
 * Models proximity explosions.
 */
class ProximityExplosion
/**
 * Constructor.
 *
 * @param epicentre The epicentre of the explosion.
 * @param radius    The proximity in which bodies are effected.
 */(private var epicentre: Vectors2D, private val proximity: Int) : Explosion {
    /**
     * Sets the epicentre to a different coordinate.
     *
     * @param v The vector position of the new epicentre.
     */
    override fun setEpicentre(v: Vectors2D) {
        epicentre = v
    }

    var bodiesEffected = ArrayList<Body>()

    /**
     * Updates the arraylist to reevaluate what bodies are effected/within the proximity.
     *
     * @param bodiesToEvaluate Arraylist of bodies in the world to check.
     */
    override fun update(bodiesToEvaluate: ArrayList<Body>) {
        bodiesEffected.clear()
        for (b in bodiesToEvaluate) {
            val blastDist = b.position.subtract(epicentre)
            if (blastDist!!.length() <= proximity) {
                bodiesEffected.add(b)
            }
        }
    }

    private val linesToBodies = ArrayList<Vectors2D?>()

    /**
     * Updates the lines to body array for the debug drawer.
     */
    fun updateLinesToBody() {
        linesToBodies.clear()
        for (b in bodiesEffected) {
            linesToBodies.add(b.position)
        }
    }

    /**
     * Debug draw method for proximity and effected objects.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    override fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        g.color = paintSettings.proximity
        val circlePotion = camera.convertToScreen(epicentre)
        val proximityRadius = camera.scaleToScreenXValue(proximity.toDouble())
        g.draw(
            Ellipse2D.Double(
                circlePotion.x - proximityRadius,
                circlePotion.y - proximityRadius,
                2 * proximityRadius,
                2 * proximityRadius
            )
        )
        updateLinesToBody()
        for (p in linesToBodies) {
            g.color = paintSettings.linesToObjects
            val worldCoord = camera.convertToScreen(p)
            g.draw(Line2D.Double(circlePotion.x, circlePotion.y, worldCoord.x, worldCoord.y))
            val lineToRadius = camera.scaleToScreenXValue(paintSettings.CIRCLE_RADIUS.toDouble())
            g.fill(
                Ellipse2D.Double(
                    worldCoord.x - lineToRadius,
                    worldCoord.y - lineToRadius,
                    2 * lineToRadius,
                    2 * lineToRadius
                )
            )
        }
    }

    /**
     * Applies blast impulse to all effected bodies centre of mass.
     *
     * @param blastPower Blast magnitude.
     */
    override fun applyBlastImpulse(blastPower: Double) {
        for (b in bodiesEffected) {
            val blastDir = b.position.subtract(epicentre)
            val distance = blastDir!!.length()
            if (distance == 0.0) return

            //Not physically correct as it should be blast * radius to object ^ 2 as the pressure of an explosion in 2D dissipates
            val invDistance = 1 / distance
            val impulseMag = blastPower * invDistance
            b.applyLinearImpulseToCentre(blastDir.normalize().scalar(impulseMag))
        }
    }
}