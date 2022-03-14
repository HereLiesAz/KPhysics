package de.chaffic.collision.bodies

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.geometry.Shape
import de.chaffic.geometry.interfaces.Translatable
import de.chaffic.math.Vec2

interface CollisionBodyInterface : Translatable {
    var shape: Shape
    override var position: Vec2
    var dynamicFriction: Double
    var staticFriction: Double
    var orientation: Double
    var aabb: AxisAlignedBoundingBox
}