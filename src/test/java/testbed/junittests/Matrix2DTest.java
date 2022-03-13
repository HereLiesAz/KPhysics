package testbed.junittests;

import library.math.Matrix2D;
import library.math.Vectors2D;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

public class Matrix2DTest {
    @Test
    public void setUsingRadians() {
        Matrix2D m = new Matrix2D();
        m.set(1);
        assertEquals(m.row1.getX(), 0.5403023058681398);
        assertEquals(m.row2.getX(), 0.8414709848078965);
        assertEquals(m.row1.getY(), -0.8414709848078965);
        assertEquals(m.row2.getY(), 0.5403023058681398);
    }

    @Test
    public void setUsingMatrix() {
        Matrix2D m = new Matrix2D();
        m.set(1);
        Matrix2D u = new Matrix2D();
        u.set(m);
        assertEquals(u.row1.getX(), m.row1.getX());
        assertEquals(u.row2.getX(), m.row2.getX());
        assertEquals(u.row1.getY(), m.row1.getY());
        assertEquals(u.row2.getY(), m.row2.getY());
    }

    @Test
    public void transpose() {
        Matrix2D m = new Matrix2D();
        m.set(1);
        Matrix2D u = new Matrix2D();
        u.set(m);
        assertEquals(u.row1.getX(), m.row1.getX());
        assertEquals(u.row2.getX(), m.row2.getX());
        assertEquals(u.row1.getY(), m.row1.getY());
        assertEquals(u.row2.getY(), m.row2.getY());
    }

    @Test
    public void mul() {
        Matrix2D m = new Matrix2D();
        m.set(1);
        Vectors2D v = new Vectors2D(1, 0);
        m.mul(v);
        assertEquals(v.getX(), 0.5403023058681398);
        assertEquals(v.getY(), 0.8414709848078965);
    }

    @Test
    public void testMul() {
        Matrix2D m = new Matrix2D();
        m.set(1);
        Vectors2D v = new Vectors2D(1, 0);
        Vectors2D q = new Vectors2D(10, 0);
        m.mul(v, q);
        assertEquals(q.getX(), 0.5403023058681398);
        assertEquals(q.getY(), 0.8414709848078965);
        assertEquals(v.getX(), 1.0);
        assertEquals(v.getY(), 0.0);
    }
}