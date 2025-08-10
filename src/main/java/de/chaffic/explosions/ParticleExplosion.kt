package de.chaffic.explosions

import de.chaffic.dynamics.Body
import de.chaffic.dynamics.World
import de.chaffic.geometry.Circle
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2

/**
 * Simulates an explosion by creating a number of small, fast-moving particles.
 *
 * This class creates a set of particle bodies and launches them outwards from an epicenter.
 * These particles can then collide with other objects in the world, transferring momentum
 * and simulating the effect of shrapnel.
 *
 * Note: This class does not implement the [Explosion] interface directly, as its mechanism
 * is different from the impulse-based explosions.
 *
 * Example of creating a particle explosion:
 * ```kotlin
 * // Create a particle explosion with 50 particles
 * val particleExplosion = ParticleExplosion(Vec2(300.0, 300.0), 50, 2.0)
 *
 * // Create the particles in the world
 * particleExplosion.createParticles(size = 2.0, density = 10, radius = 10, world = world)
 *
 * // Apply an initial impulse to the particles
 * particleExplosion.applyBlastImpulse(100.0)
 * ```
 *
 * @property particles A list of the particle bodies created by the explosion.
 *
 * @param epicentre The center point from which particles will be spawned.
 * @param noOfParticles The total number of particles to create.
 * @param lifespan The life time of the particle (currently not implemented).
 */
class ParticleExplosion(private val epicentre: Vec2, private val noOfParticles: Int, private val lifespan: Double) {
    val particles = MutableList(noOfParticles) { Body(Circle(.0),.0, .0) }

    /**
     * Creates the particle bodies and adds them to the specified world.
     * The particles are arranged in a circle around the epicenter.
     *
     * @param size The radius of each particle.
     * @param density The density of each particle.
     * @param radius The radius of the circle on which particles are initially placed.
     * @param world The world to add the particles to.
     */
    fun createParticles(size: Double, density: Int, radius: Int, world: World) {
        val separationAngle = 6.28319 / noOfParticles
        val distanceFromCentre = Vec2(.0, radius.toDouble())
        val rotate = Mat2(separationAngle)
        for (i in 0 until noOfParticles) {
            val particlePlacement = epicentre.plus(distanceFromCentre)
            val b = Body(Circle(size), particlePlacement.x, particlePlacement.y)
            b.density = density.toDouble()
            b.restitution = 1.0
            b.staticFriction = 0.0
            b.dynamicFriction = 0.0
            b.affectedByGravity = false
            b.linearDampening = 0.0
            b.particle = true
            world.addBody(b)
            particles[i] = b
            rotate.mul(distanceFromCentre)
        }
    }

    /**
     * Applies an initial outward impulse to all particles, sending them flying from the epicenter.
     *
     * @param blastPower The magnitude of the impulse to apply to each particle.
     */
    fun applyBlastImpulse(blastPower: Double) {
        var line: Vec2
        for (b in particles) {
            line = b.position.minus(epicentre)
            b.velocity.set(line.scalar(blastPower))
        }
    }
}