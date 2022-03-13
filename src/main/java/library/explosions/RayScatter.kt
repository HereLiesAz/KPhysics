package library.explosions

import demo.Camera
import demo.ColourSettings
import library.dynamics.Body
import library.math.Mat2
import library.math.Vec2
import library.rays.Ray
import java.awt.Graphics2D

/**
 * Models rayscatter explosions.
 */
class RayScatter(epicentre: Vec2, private val noOfRays: Int) {
    /**
     * Getter for rays.
     *
     * @return Array of all rays part of the ray scatter.
     */
    val rays = mutableListOf<Ray>()
    var epicentre: Vec2 = epicentre
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
        val direction = Vec2(1.0, 1.0)
        val u = Mat2(angle)
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