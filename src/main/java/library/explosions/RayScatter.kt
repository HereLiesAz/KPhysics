package library.explosions

import library.dynamics.Body
import library.math.Matrix2D
import library.math.Vectors2D
import library.rays.Ray
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D

/**
 * Models rayscatter explosions.
 */
class RayScatter(epicentre: Vectors2D, private val noOfRays: Int) {
    /**
     * Getter for rays.
     *
     * @return Array of all rays part of the ray scatter.
     */
    val rays = mutableListOf<Ray>()
    var epicentre: Vectors2D = epicentre
        set(value) {
        field = value
        for (ray in rays) {
            ray.setStartPoint(field)
        }
    }

    /**
     * Casts rays in 360 degrees with equal spacing.
     *
     * @param distance Distance of projected rays.
     */
    fun castRays(distance: Int) {
        val angle = 6.28319 / noOfRays
        val direction = Vectors2D(1.0, 1.0)
        val u = Matrix2D(angle)
        for (i in rays.indices) {
            rays.add(Ray(epicentre, direction, distance))
            u.mul(direction)
        }
    }

    /**
     * Updates all rays.
     *
     * @param worldBodies Arraylist of all bodies to update ray projections for.
     */
    fun updateRays(worldBodies: ArrayList<Body>) {
        for (ray in rays) {
            ray.updateProjection(worldBodies)
        }
    }

    /**
     * Debug draw method for rays and intersections.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        for (ray in rays) {
            ray.draw(g, paintSettings, camera)
        }
    }
}