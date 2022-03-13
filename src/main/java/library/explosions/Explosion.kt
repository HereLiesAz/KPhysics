package library.explosions

import library.dynamics.Body
import library.math.Vec2
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D

/**
 * Interface detailing what explosions need to include.
 */
interface Explosion {
    /**
     * Applies a blast impulse to the effected bodies.
     *
     * @param blastPower The impulse magnitude.
     */
    fun applyBlastImpulse(blastPower: Double)

    /**
     * Debug draw method for explosion and the effected objects.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera)

    /**
     * Updates the arraylist to reevaluate what objects are effected/within the proximity.
     *
     * @param bodiesToEvaluate Arraylist of bodies in the world to check.
     */
    fun update(bodiesToEvaluate: ArrayList<Body>)

    /**
     * Sets the epicentre to a different coordinate.
     *
     * @param v The vector position of the new epicentre.
     */
    fun setEpicentre(v: Vec2)
}