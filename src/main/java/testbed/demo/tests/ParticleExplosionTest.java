package testbed.demo.tests;

import library.dynamics.World;
import library.math.Vec2;
import testbed.demo.TestBedWindow;

public class ParticleExplosionTest {
    public static final String[] text = {"Particle Explosions:", "Left click: casts an explosion"};
    public static boolean active = false;

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        testBedWindow.setCamera(new Vec2(0, 300), 2.0);
        active = true;

        testBedWindow.buildExplosionDemo();
    }
}