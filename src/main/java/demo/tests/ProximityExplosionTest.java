package demo.tests;

import demo.window.TestBedWindow;
import library.dynamics.World;
import library.explosions.ProximityExplosion;
import library.math.Vec2;

public class ProximityExplosionTest {
    public static final String[] text = {"Proximity Explosions:", "Left click: casts an explosion"};
    public static boolean active = false;
    public static ProximityExplosion p;

    public static void load(TestBedWindow testBedWindow) {
        active = true;
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        testBedWindow.setCamera(new Vec2(0, 300), 2.0);

        testBedWindow.buildExplosionDemo();

        p = new ProximityExplosion(new Vec2(), 200);
        testBedWindow.add(p);

    }
}