package demo.tests;

import demo.window.TestBedWindow;
import library.dynamics.Body;
import library.dynamics.World;
import library.geometry.Circle;
import library.joints.Joint;
import library.joints.JointToPoint;
import library.math.Vec2;

public class NewtonsCradle {
    public static final String[] text = {"Newtons Cradle:"};

    public static void load(TestBedWindow testBedWindow) {
        testBedWindow.setWorld(new World(new Vec2(0, -9.81)));
        World temp = testBedWindow.getWorld();
        testBedWindow.setCamera(new Vec2(), 2);

        double radius = 40.0;
        int noOfCircles = 8;
        double spread = ((noOfCircles - 1) * 80 / 2.0);

        double minX, maxX;
        minX = -spread + 40;

        {
            for (int i = 0; i < noOfCircles; i++) {
                double x = minX + (i * 80);
                Body b = new Body(new Circle(radius), x, -100);
                setBody(temp, x, b);
            }
        }

        {
            minX -= 80;
            Body b = new Body(new Circle(radius), minX - 300, 200);
            setBody(temp, minX, b);
        }
    }

    private static void setBody(World temp, double x, Body b) {
        b.setRestitution(1);
        b.setStaticFriction(0);
        b.setDynamicFriction(0);
        temp.addBody(b);

        Joint j = new JointToPoint(b, new Vec2(x, 200), 300, 200000, 1000, true, new Vec2());
        temp.addJoint(j);
    }
}
