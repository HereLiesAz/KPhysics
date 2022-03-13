package library.collision

import library.dynamics.Body
import library.dynamics.Settings
import library.geometry.Circle
import library.geometry.Polygon
import library.math.Vectors2D

/**
 * Creates manifolds to detect collisions and apply forces to them. Discrete in nature and only evaluates pairs of bodies in a single manifold.
 */
class Arbiter(
    /**
     * Getter for Body A.
     *
     * @return Body A
     */
    val a: Body,
    /**
     * Getter for Body B.
     *
     * @return Body B
     */
    val b: Body
) {

    /**
     * Static fiction constant to be set during the construction of the arbiter.
     */
    var staticFriction: Double

    /**
     * Dynamic fiction constant to be set during the construction of the arbiter.
     */
    var dynamicFriction: Double

    /**
     * Array to save the contact points of the objects body's in world space.
     */
    @JvmField
    val contacts = arrayOf(Vectors2D(), Vectors2D())
    @JvmField
    var contactNormal = Vectors2D()
    @JvmField
    var contactCount = 0
    var restitution = 0.0

    /**
     * Conducts a narrow phase detection and creates a contact manifold.
     */
    fun narrowPhase() {
        restitution = Math.min(a.restitution, b.restitution)
        if (a.shape is Circle && b.shape is Circle) {
            circleVsCircle()
        } else if (a.shape is Circle && b.shape is Polygon) {
            circleVsPolygon(a, b)
        } else if (a.shape is Polygon && b.shape is Circle) {
            circleVsPolygon(b, a)
            if (contactCount > 0) {
                contactNormal.negative()
            }
        } else if (a.shape is Polygon && b.shape is Polygon) {
            polygonVsPolygon()
        }
    }

    private var penetration = 0.0

    /**
     * Main constructor for arbiter that takes two bodies to be evaluated. Sets static and dynamic friction constants here.
     *
     * @param a First body of arbiter.
     * @param b Second body of arbiter.
     */
    init {
        staticFriction = (a.staticFriction + b.staticFriction) / 2
        dynamicFriction = (a.dynamicFriction + b.dynamicFriction) / 2
    }

    /**
     * Circle vs circle collision detection method
     */
    private fun circleVsCircle() {
        val ca = a.shape as Circle
        val cb = b.shape as Circle
        val normal = b.position.subtract(a.position)
        val distance = normal.length()
        val radius = ca.radius + cb.radius
        if (distance >= radius) {
            contactCount = 0
            return
        }
        contactCount = 1
        if (distance == 0.0) {
            penetration = radius
            contactNormal = Vectors2D(.0, 1.0)
            contacts[0].set(a.position)
        } else {
            penetration = radius - distance
            contactNormal = normal.normalize()
            contacts[0].set(contactNormal.scalar(ca.radius).addi(a.position))
        }
    }

    /**
     * Circle vs Polygon collision detection method
     *
     * @param a Circle object
     * @param b Polygon Object
     */
    private fun circleVsPolygon(a: Body, b: Body) {
        val A = a.shape as Circle
        val B = b.shape as Polygon

        //Transpose effectively removes the rotation thus allowing the OBB vs OBB detection to become AABB vs OBB
        val distOfBodies = a.position.subtract(b.position)
        val polyToCircleVec = B.orient.transpose().mul(distOfBodies)
        var penetration = -Double.MAX_VALUE
        var faceNormalIndex = 0

        //Applies SAT to check for potential penetration
        //Retrieves best face of polygon
        for (i in B.vertices.indices) {
            val v = polyToCircleVec.subtract(B.vertices[i])
            val distance = B.normals[i].dotProduct(v)

            //If circle is outside of polygon, no collision detected.
            if (distance > A.radius) {
                return
            }
            if (distance > penetration) {
                faceNormalIndex = i
                penetration = distance
            }
        }

        //Get vertex's of best face
        val vector1 = B.vertices[faceNormalIndex]
        val vector2 = B.vertices[if (faceNormalIndex + 1 < B.vertices.size) faceNormalIndex + 1 else 0]
        val v1ToV2 = vector2.subtract(vector1)
        val circleBodyTov1 = polyToCircleVec.subtract(vector1)
        val firstPolyCorner = circleBodyTov1.dotProduct(v1ToV2)

        //If first vertex is positive, v1 face region collision check
        if (firstPolyCorner <= 0.0) {
            val distBetweenObj = polyToCircleVec.distance(vector1)

            //Check to see if vertex is within the circle
            if (distBetweenObj >= A.radius) {
                return
            }
            this.penetration = A.radius - distBetweenObj
            contactCount = 1
            B.orient.mul(contactNormal.set(vector1.subtract(polyToCircleVec).normalize()))
            contacts[0] = B.orient.mul(vector1, Vectors2D()).addi(b.position)
            return
        }
        val v2ToV1 = vector1.subtract(vector2)
        val circleBodyTov2 = polyToCircleVec.subtract(vector2)
        val secondPolyCorner = circleBodyTov2.dotProduct(v2ToV1)

        //If second vertex is positive, v2 face region collision check
        //Else circle has made contact with the polygon face.
        if (secondPolyCorner < 0.0) {
            val distBetweenObj = polyToCircleVec.distance(vector2)

            //Check to see if vertex is within the circle
            if (distBetweenObj >= A.radius) {
                return
            }
            this.penetration = A.radius - distBetweenObj
            contactCount = 1
            B.orient.mul(contactNormal.set(vector2.subtract(polyToCircleVec).normalize()))
            contacts[0] = B.orient.mul(vector2, Vectors2D()).addi(b.position)
        } else {
            val distFromEdgeToCircle = polyToCircleVec.subtract(vector1).dotProduct(B.normals[faceNormalIndex])
            if (distFromEdgeToCircle >= A.radius) {
                return
            }
            this.penetration = A.radius - distFromEdgeToCircle
            contactCount = 1
            B.orient.mul(B.normals[faceNormalIndex], contactNormal)
            val circleContactPoint = a.position.addi(contactNormal.negative().scalar(A.radius))
            contacts[0].set(circleContactPoint)
        }
    }

    /**
     * Polygon collision check
     */
    private fun polygonVsPolygon() {
        val pa = a.shape as Polygon
        val pb = b.shape as Polygon
        val aData = AxisData()
        findAxisOfMinPenetration(aData, pa, pb)
        if (aData.penetration >= 0) {
            return
        }
        val bData = AxisData()
        findAxisOfMinPenetration(bData, pb, pa)
        if (bData.penetration >= 0) {
            return
        }
        val referenceFaceIndex: Int
        val referencePoly: Polygon
        val incidentPoly: Polygon
        val flip: Boolean
        if (selectionBias(aData.penetration, bData.penetration)) {
            referencePoly = pa
            incidentPoly = pb
            referenceFaceIndex = aData.referenceFaceIndex
            flip = false
        } else {
            referencePoly = pb
            incidentPoly = pa
            referenceFaceIndex = bData.referenceFaceIndex
            flip = true
        }
        val incidentFaceVertexes = arrayOfNulls<Vectors2D>(2)
        var referenceNormal = referencePoly.normals[referenceFaceIndex]

        //Reference face of reference polygon in object space of incident polygon
        referenceNormal = referencePoly.orient.mul(referenceNormal, Vectors2D())
        referenceNormal = incidentPoly.orient.transpose().mul(referenceNormal, Vectors2D())

        //Finds face of incident polygon angled best vs reference poly normal.
        //Best face is the incident face that is the most anti parallel (most negative dot product)
        var incidentIndex = 0
        var minDot = Double.MAX_VALUE
        for (i in incidentPoly.vertices.indices) {
            val dot = referenceNormal.dotProduct(incidentPoly.normals[i])
            if (dot < minDot) {
                minDot = dot
                incidentIndex = i
            }
        }

        //Incident faces vertexes in world space
        incidentFaceVertexes[0] =
            incidentPoly.orient.mul(incidentPoly.vertices[incidentIndex], Vectors2D()).addi(incidentPoly.body.position)
        incidentFaceVertexes[1] = incidentPoly.orient.mul(
            incidentPoly.vertices[if (incidentIndex + 1 >= incidentPoly.vertices.size) 0 else incidentIndex + 1],
            Vectors2D()
        ).addi(incidentPoly.body.position)

        //Gets vertex's of reference polygon reference face in world space
        var v1 = referencePoly.vertices[referenceFaceIndex]
        var v2 =
            referencePoly.vertices[if (referenceFaceIndex + 1 == referencePoly.vertices.size) 0 else referenceFaceIndex + 1]

        //Rotate and translate vertex's of reference poly
        v1 = referencePoly.orient.mul(v1, Vectors2D()).addi(referencePoly.body.position)
        v2 = referencePoly.orient.mul(v2, Vectors2D()).addi(referencePoly.body.position)
        val refTangent = v2.subtract(v1)
        refTangent.normalize()
        val negSide = -refTangent.dotProduct(v1)
        val posSide = refTangent.dotProduct(v2)
        // Clips the incident face against the reference
        var np = clip(refTangent.negativeVec(), negSide, incidentFaceVertexes)
        if (np < 2) {
            return
        }
        np = clip(refTangent, posSide, incidentFaceVertexes)
        if (np < 2) {
            return
        }
        val refFaceNormal = refTangent.normal().negativeVec()
        val contactVectorsFound = arrayOfNulls<Vectors2D>(2)
        var totalPen = 0.0
        var contactsFound = 0

        //Discards points that are positive/above the reference face
        for (i in 0..1) {
            val separation = refFaceNormal.dotProduct(incidentFaceVertexes[i]) - refFaceNormal.dotProduct(v1)
            if (separation <= 0.0 + Settings.EPSILON) {
                contactVectorsFound[contactsFound] = incidentFaceVertexes[i]
                totalPen += -separation
                contactsFound++
            }
        }
        val contactPoint: Vectors2D?
        if (contactsFound == 1) {
            contactPoint = contactVectorsFound[0]
            penetration = totalPen
        } else {
            contactPoint = contactVectorsFound[1]!!.addi(contactVectorsFound[0]).scalar(0.5)
            penetration = totalPen / 2
        }
        contactCount = 1
        contacts[0].set(contactPoint)
        contactNormal.set(if (flip) refFaceNormal.negative() else refFaceNormal)
    }

    /**
     * Clipping for polygon collisions. Clips incident face against side planes of the reference face.
     *
     * @param planeTangent Plane to clip against
     * @param offset       Offset for clipping in world space to incident face.
     * @param incidentFace Clipped face vertex's
     * @return Number of clipped vertex's
     */
    private fun clip(planeTangent: Vectors2D, offset: Double, incidentFace: Array<Vectors2D?>): Int {
        var num = 0
        val out = arrayOf(
            Vectors2D(incidentFace[0]),
            Vectors2D(incidentFace[1])
        )
        val dist = planeTangent.dotProduct(incidentFace[0]) - offset
        val dist1 = planeTangent.dotProduct(incidentFace[1]) - offset
        if (dist <= 0.0) out[num++].set(incidentFace[0])
        if (dist1 <= 0.0) out[num++].set(incidentFace[1])
        if (dist * dist1 < 0.0) {
            val interp = dist / (dist - dist1)
            out[num].set(incidentFace[1]!!.subtract(incidentFace[0]).scalar(interp).addi(incidentFace[0]))
            num++
        }
        incidentFace[0] = out[0]
        incidentFace[1] = out[1]
        return num
    }

    /**
     * Finds the incident face of polygon A in object space relative to polygons B position.
     *
     * @param data Data obtained from earlier penetration test.
     * @param A    Polygon A to test.
     * @param B    Polygon B to test.
     */
    fun findAxisOfMinPenetration(data: AxisData, A: Polygon, B: Polygon) {
        var distance = -Double.MAX_VALUE
        var bestIndex = 0
        for (i in A.vertices.indices) {
            //Applies polygon A's orientation to its normals for calculation.
            val polyANormal = A.orient.mul(A.normals[i], Vectors2D())

            //Rotates the normal by the clock wise rotation matrix of B to put the normal relative to the object space of polygon B
            //Polygon b is axis aligned and the normal is located according to this in the correct position in object space
            val objectPolyANormal = B.orient.transpose().mul(polyANormal, Vectors2D())
            var bestProjection = Double.MAX_VALUE
            var bestVertex = B.vertices[0]

            //Finds the index of the most negative vertex relative to the normal of polygon A
            for (x in B.vertices.indices) {
                val vertex = B.vertices[x]
                val projection = vertex.dotProduct(objectPolyANormal)
                if (projection < bestProjection) {
                    bestVertex = vertex
                    bestProjection = projection
                }
            }

            //Distance of B to A in world space space
            val distanceOfBA = A.body.position.subtract(B.body.position)

            //Best vertex relative to polygon B in object space
            val polyANormalVertex =
                B.orient.transpose().mul(A.orient.mul(A.vertices[i], Vectors2D()).addi(distanceOfBA))

            //Distance between best vertex and polygon A's plane in object space
            val d = objectPolyANormal.dotProduct(bestVertex.subtract(polyANormalVertex))

            //Records penetration and vertex
            if (d > distance) {
                distance = d
                bestIndex = i
            }
        }
        data.penetration = distance
        data.referenceFaceIndex = bestIndex
    }

    /**
     * Resolves any penetrations that are left overlapping between shapes. This can be cause due to integration errors of the solvers integration method.
     * Based on linear projection to move the shapes away from each other based on a correction constant and scaled relative to the inverse mass of the objects.
     */
    fun penetrationResolution() {
        val penetrationTolerance = penetration - Settings.PENETRATION_ALLOWANCE
        if (penetrationTolerance <= 0.0) {
            return
        }
        val totalMass = a.mass + b.mass
        val correction = penetrationTolerance * Settings.PENETRATION_CORRECTION / totalMass
        a.position = a.position.addi(contactNormal.scalar(-a.mass * correction))
        b.position = b.position.addi(contactNormal.scalar(b.mass * correction))
    }

    /**
     * Solves the current contact manifold and applies impulses based on any contacts found.
     */
    fun solve() {
        val contactA = contacts[0].subtract(a.position)
        val contactB = contacts[0].subtract(b.position)

        //Relative velocity created from equation found in GDC talk of box2D lite.
        var relativeVel = b.velocity.addi(contactB.crossProduct(b.angularVelocity)).subtract(a.velocity).subtract(
            contactA.crossProduct(
                a.angularVelocity
            )
        )

        //Positive = converging Negative = diverging
        val contactVel = relativeVel.dotProduct(contactNormal)

        //Prevents objects colliding when they are moving away from each other.
        //If not, objects could still be overlapping after a contact has been resolved and cause objects to stick together
        if (contactVel >= 0) {
            return
        }
        val acn = contactA.crossProduct(contactNormal)
        val bcn = contactB.crossProduct(contactNormal)
        val inverseMassSum = a.invMass + b.invMass + acn * acn * a.invI + bcn * bcn * b.invI
        var j = -(restitution + 1) * contactVel
        j /= inverseMassSum
        val impulse = contactNormal.scalar(j)
        b.applyLinearImpulse(impulse, contactB)
        a.applyLinearImpulse(impulse.negativeVec(), contactA)
        relativeVel = b.velocity.addi(contactB.crossProduct(b.angularVelocity)).subtract(a.velocity).subtract(
            contactA.crossProduct(
                a.angularVelocity
            )
        )
        val t = relativeVel.copy()
        t.add(contactNormal.scalar(-relativeVel.dotProduct(contactNormal))).normalize()
        var jt = -relativeVel.dotProduct(t)
        jt /= inverseMassSum
        val tangentImpulse: Vectors2D
        tangentImpulse = if (StrictMath.abs(jt) < j * staticFriction) {
            t.scalar(jt)
        } else {
            t.scalar(j).scalar(-dynamicFriction)
        }
        b.applyLinearImpulse(tangentImpulse, contactB)
        a.applyLinearImpulse(tangentImpulse.negativeVec(), contactA)
    }

    companion object {
        /**
         * Method to check if point is inside a body in world space.
         *
         * @param b          Body to check against.
         * @param startPoint Vector point to check if its inside the first body.
         * @return boolean value whether the point is inside the first body.
         */
        @JvmStatic
        fun isPointInside(b: Body, startPoint: Vectors2D): Boolean {
            if (b.shape is Polygon) {
                val poly = b.shape as Polygon
                for (i in poly.vertices.indices) {
                    val objectPoint = startPoint.subtract(
                        poly.body.position.addi(
                            poly.body.shape.orient.mul(
                                poly.vertices[i],
                                Vectors2D()
                            )
                        )
                    )
                    if (objectPoint.dotProduct(poly.body.shape.orient.mul(poly.normals[i], Vectors2D())) > 0) {
                        return false
                    }
                }
            } else if (b.shape is Circle) {
                val circle = b.shape as Circle
                val d = b.position.subtract(startPoint)
                return d.length() <= circle.radius
            }
            return true
        }

        /**
         * Selects one value over another. Intended for polygon collisions to aid in choosing which axis of separation intersects the other in a consistent manner.
         * Floating point error can occur in the rotation calculations thus this method helps with choosing one axis over another in a consistent manner for stability.
         *
         * @param a penetration value a
         * @param b penetration value b
         * @return boolean value whether a is to be preferred or not.
         */
        private fun selectionBias(a: Double, b: Double): Boolean {
            return a >= b * Settings.BIAS_RELATIVE + a * Settings.BIAS_ABSOLUTE
        }
    }
}

/**
 * Class for data related to axis
 */
class AxisData {
    /**
     * Gets the penetration value stored.
     *
     * @return double penetration value.
     */
    /**
     * Sets penetration value.
     *
     * @param value Penetration value of type double.
     */
    var penetration: Double
    /**
     * Gets the referenceFaceIndex value stored
     *
     * @return int referenceFaceIndex value.
     */
    /**
     * Sets the reference face index variable to an int value.
     *
     * @param value Value to set index variable to.
     */
    var referenceFaceIndex: Int

    /**
     * Default constructor
     */
    init {
        penetration = -Double.MAX_VALUE
        referenceFaceIndex = 0
    }
}