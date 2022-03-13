package library.dynamics

import library.collision.AABB
import library.collision.Arbiter
import library.joints.Joint
import library.math.Vectors2D
import testbed.Camera
import testbed.ColourSettings
import java.awt.Graphics2D
import java.awt.geom.Line2D

/**
 * Class for creating a world with iterative solver structure.
 */
class World {
    private var gravity: Vectors2D

    /**
     * Constructor
     *
     * @param gravity The strength of gravity in the world.
     */
    constructor(gravity: Vectors2D) {
        this.gravity = gravity
    }

    /**
     * Default constructor
     */
    constructor() {
        gravity = Vectors2D(.0, .0)
    }

    /**
     * Sets gravity.
     *
     * @param gravity The strength of gravity in the world.
     */
    fun setGravity(gravity: Vectors2D) {
        this.gravity = gravity
    }

    @kotlin.jvm.JvmField
    var bodies = ArrayList<Body?>()

    /**
     * Adds a body to the world
     *
     * @param b Body to add.
     * @return Returns the newly added body.
     */
    fun addBody(b: Body?): Body? {
        bodies.add(b)
        return b
    }

    /**
     * Removes a body from the world.
     *
     * @param b The body to remove from the world.
     */
    fun removeBody(b: Body?) {
        bodies.remove(b)
    }

    @kotlin.jvm.JvmField
    var joints = ArrayList<Joint>()

    /**
     * Adds a joint to the world.
     *
     * @param j The joint to add.
     * @return Returns the joint added to the world.
     */
    fun addJoint(j: Joint): Joint {
        joints.add(j)
        return j
    }

    /**
     * Removes a joint from the world.
     *
     * @param j The joint to remove from the world.
     */
    fun removeJoint(j: Joint) {
        joints.remove(j)
    }

    var contacts = ArrayList<Arbiter>()

    /**
     * The main time step method for the world to conduct an iteration of the current world call this method with a desired time step value.
     *
     * @param dt Timestep
     */
    fun step(dt: Double) {
        contacts.clear()
        broadPhaseCheck()
        semiImplicit(dt)

        //Correct positional errors from the discrete collisions
        for (contact in contacts) {
            contact.penetrationResolution()
        }
    }

    /**
     * Semi implicit euler integration method for the world bodies and forces.
     *
     * @param dt Timestep
     */
    private fun semiImplicit(dt: Double) {
        //Applies tentative velocities
        applyForces(dt)
        solve(dt)

        //Integrate positions
        for (b in bodies) {
            if (b!!.invMass == 0.0) {
                continue
            }
            b.position.add(b.velocity.scalar(dt))
            b.orientation = b.orientation + dt * b.angularVelocity
            b.force[0.0] = 0.0
            b.torque = 0.0
        }
    }

    /**
     * Applies semi-implicit euler and drag forces.
     *
     * @param dt Timestep
     */
    private fun applyForces(dt: Double) {
        for (b in bodies) {
            if (b!!.invMass == 0.0) {
                continue
            }
            applyLinearDrag(b)
            if (b.affectedByGravity) {
                b.velocity.add(gravity.scalar(dt))
            }
            b.velocity.add(b.force.scalar(b.invMass).scalar(dt))
            b.angularVelocity += dt * b.invI * b.torque
        }
    }

    /**
     * Method to apply all forces in the world.
     *
     * @param dt Timestep
     */
    private fun solve(dt: Double) {
        /*
        Resolve joints
        Note: this is removed from the iterations at this stage as the application of forces is different.
        The extra iterations on joints make the forces of the joints multiple times larger equal to the number of iterations.
        Early out could be used like in the collision solver
        This may change in the future and will be revised at a later date.
        */
        for (j in joints) {
            j.applyTension()
        }

        //Resolve collisions
        for (i in 0 until Settings.ITERATIONS) {
            for (contact in contacts) {
                contact.solve()
            }
        }
    }

    /**
     * Applies linear drag to a body.
     *
     * @param b Body to apply drag to.
     */
    private fun applyLinearDrag(b: Body?) {
        val velocityMagnitude = b!!.velocity.length()
        val dragForceMagnitude = velocityMagnitude * velocityMagnitude * b.linearDampening
        val dragForceVector = b.velocity.normalized.scalar(-dragForceMagnitude)
        b.applyForceToCentre(dragForceVector)
    }

    /**
     * A discrete Broad phase check of collision detection.
     */
    private fun broadPhaseCheck() {
        for (i in bodies.indices) {
            val a = bodies[i]
            for (x in i + 1 until bodies.size) {
                val b = bodies[x]

                //Ignores static or particle objects
                if (a!!.invMass == 0.0 && b!!.invMass == 0.0 || a.particle && b!!.particle) {
                    continue
                }
                if (AABB.aabbOverlap(a, b!!)) {
                    narrowPhaseCheck(a, b)
                }
            }
        }
    }

    /**
     * If broad phase detection check passes, a narrow phase check is conducted to determine for certain if two objects are intersecting.
     * If two objects are, arbiters of contacts found are generated
     *
     * @param a
     * @param b
     */
    private fun narrowPhaseCheck(a: Body?, b: Body?) {
        val contactQuery = Arbiter(a!!, b!!)
        contactQuery.narrowPhase()
        if (contactQuery.contactCount > 0) {
            contacts.add(contactQuery)
        }
    }

    /**
     * Clears all objects in the current world
     */
    fun clearWorld() {
        bodies.clear()
        contacts.clear()
        joints.clear()
    }

    /**
     * Applies gravitational forces between to objects (force applied to centre of body)
     */
    fun gravityBetweenObj() {
        for (a in bodies.indices) {
            val A = bodies[a]
            for (b in a + 1 until bodies.size) {
                val B = bodies[b]
                val distance = A!!.position.distance(B!!.position)
                val force = Math.pow(6.67, -11.0) * A.mass * B.mass / (distance * distance)
                var direction: Vectors2D? = Vectors2D(B.position.x - A.position.x, B.position.y - A.position.y)
                direction = direction!!.scalar(force)
                val oppositeDir = Vectors2D(-direction.x, -direction.y)
                A.force.addi(direction)
                B.force.addi(oppositeDir)
            }
        }
    }

    /**
     * Debug draw method for world objects.
     *
     * @param g             Graphics2D object to draw to
     * @param paintSettings Colour settings to draw the objects to screen with
     * @param camera        Camera class used to convert points from world space to view space
     */
    fun drawContact(g: Graphics2D, paintSettings: ColourSettings, camera: Camera) {
        for (contact in contacts) {
            val point = contact.contacts[0]
            var line: Vectors2D?
            var beginningOfLine: Vectors2D
            var endOfLine: Vectors2D
            g.color = paintSettings.contactPoint
            line = contact.contactNormal.normal().scalar(paintSettings.TANGENT_LINE_SCALAR)
            beginningOfLine = camera.convertToScreen(point.addi(line))
            endOfLine = camera.convertToScreen(point.subtract(line))
            g.draw(Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y))
            line = contact.contactNormal.scalar(paintSettings.NORMAL_LINE_SCALAR)
            beginningOfLine = camera.convertToScreen(point.addi(line))
            endOfLine = camera.convertToScreen(point.subtract(line))
            g.draw(Line2D.Double(beginningOfLine.x, beginningOfLine.y, endOfLine.x, endOfLine.y))
        }
    }
}