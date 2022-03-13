package testbed.demo.tests;

import library.dynamics.World;
import library.math.Vec2;
import testbed.demo.TestBedWindow;

public class Car {
    public static final String[] text = {"Car:"};

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, 0)));
        World world = testBedWindow.getWorld();
        testBedWindow.setCamera(new Vec2(0, 0), 1.4);

    }
}
