package library.rays

import library.dynamics.Body
import library.dynamics.World
import library.geometry.Circle
import library.geometry.Polygon
import library.math.Vectors2D
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D
import java.awt.geom.Line2D

/**
 * Class to allow two polygons to be sliced.
 */
class Slice(private val startPoint: Vectors2D, direction: Vectors2D, distance: Double) {
    private var distance: Double
    private var direction: Vectors2D?
    private val intersectingBodiesInfo = ArrayList<RayInformation>()

    /**
     * Constructor to create a slice to later be evaluated.
     *
     * @param startPoint The origin of the rays projection.
     * @param direction  The direction of the ray points in radians.
     * @param distance   The distance the ray is projected
     */
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
        intersectingBodiesInfo.clear()
        val endPoint = direction!!.scalar(distance)
        val end_x = endPoint!!.x
        val end_y = endPoint.y
        val min_t1 = Double.POSITIVE_INFINITY
        var min_px = 0.0
        var min_py = 0.0
        var noOfIntersections = 0
        for (B in bodiesToEvaluate) {
            if (B.shape is Polygon) {
                val poly = B.shape as Polygon
                for (i in poly.vertices.indices) {
                    var startOfPolyEdge = poly.vertices[i]
                    var endOfPolyEdge = poly.vertices[if (i + 1 == poly.vertices.size) 0 else i + 1]
                    startOfPolyEdge = poly.orient.mul(startOfPolyEdge, Vectors2D()).addi(B.position)
                    endOfPolyEdge = poly.orient.mul(endOfPolyEdge, Vectors2D()).addi(B.position)
                    val dx = endOfPolyEdge.x - startOfPolyEdge.x
                    val dy = endOfPolyEdge.y - startOfPolyEdge.y

                    //Check to see if the lines are not parallel
                    if (dx - end_x != 0.0 && dy - end_y != 0.0) {
                        val t2 =
                            (end_x * (startOfPolyEdge.y - startPoint.y) + end_y * (startPoint.x - startOfPolyEdge.x)) / (dx * end_y - dy * end_x)
                        val t1 = (startOfPolyEdge.x + dx * t2 - startPoint.x) / end_x
                        if (t1 > 0 && t2 >= 0 && t2 <= 1.0) {
                            val point = Vectors2D(startPoint.x + end_x * t1, startPoint.y + end_y * t1)
                            val dist = point.subtract(startPoint).length()
                            if (dist < distance) {
                                min_px = point.x
                                min_py = point.y
                                intersectingBodiesInfo.add(RayInformation(B, min_px, min_py, i))
                                noOfIntersections++
                            }
                        }
                    }
                }
            } else if (B.shape is Circle) {
                val circle = B.shape as Circle
                val ray = endPoint.copy()
                val circleCenter = B.position.copy()
                val r = circle.radius
                val difInCenters = startPoint.subtract(circleCenter)
                val a = ray!!.dotProduct(ray)
                val b = 2 * difInCenters!!.dotProduct(ray)
                val c = difInCenters.dotProduct(difInCenters) - r * r
                var discriminant = b * b - 4 * a * c
                if (discriminant > 0) {
                    discriminant = Math.sqrt(discriminant)
                    val t1 = (-b - discriminant) / (2 * a)
                    if (t1 >= 0 && t1 <= 1) {
                        min_px = startPoint.x + end_x * t1
                        min_py = startPoint.y + end_y * t1
                        intersectingBodiesInfo.add(RayInformation(B, min_px, min_py, -1))
                    }
                    val t2 = (-b + discriminant) / (2 * a)
                    if (t2 >= 0 && t2 <= 1) {
                        min_px = startPoint.x + end_x * t2
                        min_py = startPoint.y + end_y * t2
                        intersectingBodiesInfo.add(RayInformation(B, min_px, min_py, -1))
                    }
                }
            }
            if (noOfIntersections % 2 == 1) {
                intersectingBodiesInfo.removeAt(intersectingBodiesInfo.size - 1)
                noOfIntersections = 0
            }
        }
    }

    /**
     * Slices any polygons in the world supplied that intersect with the ray projection.
     *
     * @param world World object for the slice to effect.
     */
    fun sliceObjects(world: World) {
        val k = intersectingBodiesInfo.size % 2
        var i = 0
        while (i < intersectingBodiesInfo.size - k) {
            val b = intersectingBodiesInfo[i].b
            val isStatic = b!!.mass == 0.0
            if (b.shape is Polygon) {
                val p = b.shape as Polygon
                val intersection1 = intersectingBodiesInfo[i]
                val intersection2 = intersectingBodiesInfo[i + 1]
                var obj1firstIndex = intersection1.index
                val secondIndex = intersection2.index
                val obj2firstIndex = obj1firstIndex
                var totalVerticesObj1 = obj1firstIndex + 2 + (p.vertices.size - secondIndex)
                val obj1Vertz = arrayOfNulls<Vectors2D>(totalVerticesObj1)
                for (x in 0 until obj1firstIndex + 1) {
                    obj1Vertz[x] = b.shape.orient.mul(p.vertices[x], Vectors2D()).addi(b.position)
                }
                obj1Vertz[++obj1firstIndex] = intersectingBodiesInfo[i].coord
                obj1Vertz[++obj1firstIndex] = intersectingBodiesInfo[i + 1].coord
                for (x in secondIndex + 1 until p.vertices.size) {
                    obj1Vertz[++obj1firstIndex] = b.shape.orient.mul(p.vertices[x], Vectors2D()).addi(b.position)
                }
                var polyCentre = findPolyCentre(obj1Vertz)
                val b1 = Body(Polygon(obj1Vertz.filterNotNull().toTypedArray()), polyCentre.x, polyCentre.y)
                if (isStatic) b1.setDensity(0.0)
                world.addBody(b1)
                totalVerticesObj1 = secondIndex - obj2firstIndex + 2
                val obj2Vertz = arrayOfNulls<Vectors2D>(totalVerticesObj1)
                var indexToAddTo = 0
                obj2Vertz[indexToAddTo++] = intersection1.coord
                for (x in obj2firstIndex + 1..secondIndex) {
                    obj2Vertz[indexToAddTo++] = b.shape.orient.mul(p.vertices[x], Vectors2D()).addi(b.position)
                }
                obj2Vertz[totalVerticesObj1 - 1] = intersection2.coord
                polyCentre = findPolyCentre(obj2Vertz)
                val b2 = Body(Polygon(obj2Vertz.filterNotNull().toTypedArray()), polyCentre.x, polyCentre.y)
                if (isStatic) b2.setDensity(0.0)
                world.addBody(b2)
            }
            world.removeBody(b)
            i += 2
        }
    }

    /**
     * Finds the center of mass of a polygon and return its.
     *
     * @param obj2Vertz Vertices of polygon to find center of mass of.
     * @return Center of mass of type Vectors2D.
     */
    private fun findPolyCentre(obj2Vertz: Array<Vectors2D?>): Vectors2D {
        var accumulatedArea = 0.0
        var centerX = 0.0
        var centerY = 0.0
        var i = 0
        var j = obj2Vertz.size - 1
        while (i < obj2Vertz.size) {
            val temp = obj2Vertz[i]!!.x * obj2Vertz[j]!!.y - obj2Vertz[j]!!.x * obj2Vertz[i]!!.y
            accumulatedArea += temp
            centerX += (obj2Vertz[i]!!.x + obj2Vertz[j]!!.x) * temp
            centerY += (obj2Vertz[i]!!.y + obj2Vertz[j]!!.y) * temp
            j = i++
        }
        if (accumulatedArea == 0.0) return Vectors2D()
        accumulatedArea *= 3.0
        return Vectors2D(centerX / accumulatedArea, centerY / accumulatedArea)
    }

    /**
     * Debug draw method for slice object.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun draw(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        g.color = paintSettings.projectedRay
        val epicenter = camera.convertToScreen(startPoint)
        val endPoint = camera.convertToScreen(direction!!.scalar(distance).addi(startPoint))
        g.draw(Line2D.Double(epicenter.x, epicenter.y, endPoint.x, endPoint.y))
        g.color = paintSettings.rayToBody
        for (i in intersectingBodiesInfo.indices) {
            if ((i + 1) % 2 == 0) {
                val intersection1 = camera.convertToScreen(intersectingBodiesInfo[i - 1].coord)
                val intersection2 = camera.convertToScreen(intersectingBodiesInfo[i].coord)
                g.draw(Line2D.Double(intersection2.x, intersection2.y, intersection1.x, intersection1.y))
            }
        }
    }

    /**
     * Sets the direction of the ray to a different value.
     *
     * @param sliceVector Desired direction value
     */
    fun setDirection(sliceVector: Vectors2D) {
        direction = sliceVector.subtract(startPoint)
        distance = direction!!.length()
        direction!!.normalize()
    }
}