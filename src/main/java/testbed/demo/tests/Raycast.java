package testbed.demo.tests;

import library.collision.Arbiter;
import library.dynamics.Body;
import library.rays.Ray;
import library.dynamics.World;
import library.math.Mat2;
import library.math.Vec2;
import testbed.demo.TestBedWindow;

public class Raycast {
    public static final String[] text = {"Raycast:"};
    public static boolean active = false;

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        testBedWindow.setCamera(new Vec2(-100, -20), 3.3);
        active = true;

        boolean isValid = false;
        while (!isValid) {
            isValid = true;
            testBedWindow.generateBoxOfObjects();
            for (Body b : testBedWindow.getWorld().getBodies()) {
                if (Arbiter.isPointInside(b, new Vec2())) {
                    isValid = false;
                    testBedWindow.getWorld().clearWorld();
                    break;
                }
            }
        }

        Ray r = new Ray(new Vec2(), new Vec2(0, 1), 1000);
        testBedWindow.add(r);
    }

    public static void action(Ray r) {
        Mat2 u = new Mat2();
        u.set(-0.0006);
        u.mul(r.getDirection());
    }
}
