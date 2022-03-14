package de.chaffic.collision.bodies

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.geometry.Shape
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2

interface CollisionBodyInterface : TranslatableBody {
    var shape: Shape
    override var position: Vec2
    var dynamicFriction: Double
    var staticFriction: Double
    var orientation: Double
    var aabb: AxisAlignedBoundingBox
}