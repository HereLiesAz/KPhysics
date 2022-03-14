package de.chaffic.dynamics

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.dynamics.bodies.PhysicalBody
import de.chaffic.geometry.Shape

/**
 * Class to create a body to add to a world.
 *
 * @param shape Shape to bind to body.
 * @param x     Position x in world space.
 * @param y     Position y in world space.
 */
class Body(var shape: Shape, x: Double, y: Double): PhysicalBody(x, y, shape) {
    var dynamicFriction: Double
    var staticFriction: Double
    var orientation: Double = 0.0
        set(value) {
            field = value
            shape.orientation.set(orientation)
            shape.createAABB()
        }
    lateinit var aabb: AxisAlignedBoundingBox

    init {
        shape.body = this
        density = density
        staticFriction = 0.5
        dynamicFriction = 0.2
        orientation = 0.0
        shape.orientation.set(orientation)
        shape.createAABB()
    }
}