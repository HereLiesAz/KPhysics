package demo.tests;

import demo.window.TestBedWindow;
import library.dynamics.Body;
import library.dynamics.World;
import library.geometry.Polygon;
import library.math.Vec2;

public class MixedShapes {
    public static final String[] text = {"Mixed Shapes:"};

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        World temp = testBedWindow.getWorld();
        testBedWindow.setCamera(new Vec2(-0, 200), 2.2);

        //Polygon containers
        {
            Body b1 = new Body(new Polygon(15.0, 100.0), -300, 0);
            b1.setDensity(0);
            temp.addBody(b1);

            Body b2 = new Body(new Polygon(15.0, 100.0), 300, 0);
            b2.setDensity(0);
            temp.addBody(b2);

            Body b3 = new Body(new Polygon(315.0, 15.0), 0, -115);
            b3.setDensity(0);
            temp.addBody(b3);
        }

        testBedWindow.generateRandomObjects(new Vec2(-280, -100), new Vec2(280, 800), 30, 80);
    }
}
