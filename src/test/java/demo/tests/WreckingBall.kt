package demo.tests;

import demo.window.TestBedWindow;
import library.dynamics.Body;
import library.dynamics.World;
import library.geometry.Circle;
import library.geometry.Polygon;
import library.joints.Joint;
import library.joints.JointToPoint;
import library.math.Vec2;

public class WreckingBall {
    public static final String[] text = {"Wrecking Ball"};

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        World temp = testBedWindow.getWorld();
        testBedWindow.setCamera(new Vec2(0, 100), 1.7);

        {
            for (int x = 0; x < 10; x++) {
                for (int y = 0; y < 10; y++) {
                    Body b = new Body(new Polygon(10.0, 10.0), 110 + (x * 20), (y * 20));
                    temp.addBody(b);
                }
            }

            Body b = new Body(new Polygon(100.0, 10.0), 200, -20);
            b.setDensity(0);
            temp.addBody(b);
        }

        {
            Body b2 = new Body(new Circle(40.0), -250, 320);
            b2.setDensity(2);
            temp.addBody(b2);

            Joint j = new JointToPoint(b2, new Vec2(0, 320), 250, 200, 100, true, new Vec2());
            temp.addJoint(j);
        }
    }
}