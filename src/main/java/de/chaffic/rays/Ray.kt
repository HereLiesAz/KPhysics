package de.chaffic.rays

import de.chaffic.dynamics.Body
import de.chaffic.geometry.Circle
import de.chaffic.geometry.Polygon
import de.chaffic.math.Vec2
import kotlin.math.sqrt

/**
 * Ray class to define and project rays in a world.
 *
 * @param startPoint The origin of the rays projection.
 * @param direction  The direction of the ray points in radians.
 * @param distance   The distance the ray is projected
 */
class Ray(var startPoint: Vec2, direction: Vec2, distance: Int) {
    val distance: Int
    /**
     * Gets the direction of the ray in radians.
     *
     * @return direction variable of type Vec2.
     */
    var direction: Vec2

    /**
     * Convenience constructor with ray set at origin. Similar to
     * [.Ray]
     *
     * @param direction The direction of the ray points in radians.
     * @param distance  The distance the ray is projected
     */
    constructor(direction: Double, distance: Int) : this(Vec2(), Vec2(direction), distance)

    /**
     * Convenience constructor with ray set at origin. Similar to
     * [.Ray]
     *
     * @param direction The direction of the ray points.
     * @param distance  The distance the ray is projected
     */
    constructor(direction: Vec2, distance: Int) : this(Vec2(), direction, distance)

    /**
     * Convenience constructor. Similar to
     * [.Ray]
     *
     * @param startPoint The origin of the rays projection.
     * @param direction  The direction of the ray points in radians.
     * @param distance   The distance the ray is projected
     */
    constructor(startPoint: Vec2, direction: Double, distance: Int) : this(
        startPoint,
        Vec2(direction),
        distance
    )

    var rayInformation: RayInformation? = null
        private set

    init {
        this.direction = direction.normalized
        this.distance = distance
    }

    /**
     * Updates the projection in world space and acquires information about the closest intersecting object with the ray projection.
     *
     * @param bodiesToEvaluate Arraylist of bodies to check if they intersect with the ray projection.
     */
    fun updateProjection(bodiesToEvaluate: ArrayList<Body>) {
        rayInformation = null
        val endPoint = direction.scalar(distance.toDouble())
        val endX = endPoint.x
        val endY = endPoint.y
        var minT1 = Double.POSITIVE_INFINITY
        var minPx = 0.0
        var minPy = 0.0
        var intersectionFound = false
        var closestBody: Body? = null
        for (B in bodiesToEvaluate) {
            if (B.shape is Polygon) {
                val poly = B.shape as Polygon
                for (i in poly.vertices.indices) {
                    var startOfPolyEdge = poly.vertices[i]
                    var endOfPolyEdge = poly.vertices[if (i + 1 == poly.vertices.size) 0 else i + 1]
                    startOfPolyEdge = poly.orientation.mul(startOfPolyEdge, Vec2()).plus(B.position)
                    endOfPolyEdge = poly.orientation.mul(endOfPolyEdge, Vec2()).plus(B.position)
                    val dx = endOfPolyEdge.x - startOfPolyEdge.x
                    val dy = endOfPolyEdge.y - startOfPolyEdge.y

                    //Check to see if the lines are not parallel
                    if (dx - endX != 0.0 && dy - endY != 0.0) {
                        val t2 =
                            (endX * (startOfPolyEdge.y - startPoint.y) + endY * (startPoint.x - startOfPolyEdge.x)) / (dx * endY - dy * endX)
                        val t1 = (startOfPolyEdge.x + dx * t2 - startPoint.x) / endX
                        if (t1 > 0 && t2 >= 0 && t2 <= 1.0) {
                            val point = Vec2(startPoint.x + endX * t1, startPoint.y + endY * t1)
                            val dist = point.minus(startPoint).length()
                            if (t1 < minT1 && dist < distance) {
                                minT1 = t1
                                minPx = point.x
                                minPy = point.y
                                intersectionFound = true
                                closestBody = B
                            }
                        }
                    }
                }
            } else if (B.shape is Circle) {
                val circle = B.shape as Circle
                val ray = endPoint.copy()
                val circleCenter = B.position.copy()
                val r = circle.radius
                val difInCenters = startPoint.minus(circleCenter)
                val a = ray.dot(ray)
                val b = 2 * difInCenters.dot(ray)
                val c = difInCenters.dot(difInCenters) - r * r
                var discriminant = b * b - 4 * a * c
                if (discriminant >= 0) {
                    discriminant = sqrt(discriminant)
                    val t1 = (-b - discriminant) / (2 * a)
                    if (t1 in 0.0..1.0) {
                        if (t1 < minT1) {
                            minT1 = t1
                            minPx = startPoint.x + endX * t1
                            minPy = startPoint.y + endY * t1
                            intersectionFound = true
                            closestBody = B
                        }
                    }
                }
            }
        }
        if (intersectionFound) {
            rayInformation = closestBody?.let { RayInformation(it, minPx, minPy, -1) }
        }
    }
}