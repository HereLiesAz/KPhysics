package demo.tests;

import demo.window.TestBedWindow;
import library.dynamics.Body;
import library.dynamics.World;
import library.geometry.Polygon;
import library.math.Vec2;

public class StackedObjects {
    public static final String[] text = {"Stacked Objects:"};

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        World temp = testBedWindow.getWorld();
        testBedWindow.setCamera(new Vec2(0, 150), 1.8);

        {
            for (int x = 0; x < 15; x++) {
                for (int y = 0; y < 20; y++) {
                    Body b = new Body(new Polygon(10.0, 10.0), -140 + (x * 20), -100 + (y * 20));
                    temp.addBody(b);
                }
            }
            for (int x = 0; x < 15; x++) {
                Body b = new Body(new Polygon(10.0, 10.0), -140 + (x * 20), 400);
                b.setDensity(10);
                temp.addBody(b);
            }

            Body b = new Body(new Polygon(150.0, 10.0), 0, -120);
            b.setDensity(0);
            temp.addBody(b);
        }
    }
}
