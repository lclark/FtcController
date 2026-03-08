package org.firstinspires.ftc.quackens;

import static junit.framework.TestCase.assertEquals;

import org.junit.Test;

public class TestUtils {
    @Test
    public void testClampDouble() {
        assertEquals(1.0, Utils.clamp(1.2, 0.1, 1.0));
        assertEquals(-1.0, Utils.clamp(-5, -1, 1.0));
        assertEquals(0.5, Utils.clamp(0.5, -1, 1.0));
    }

    @Test
    public void testClampFloat() {
        assertEquals(1.0f, Utils.clamp(1.2f, 0.1f, 1.0f));
        assertEquals(-1.0f, Utils.clamp(-5f, -1f, 1.0f));
        assertEquals(0.5f, Utils.clamp(0.5f, -1f, 1.0f));
    }

    @Test
    public void testRotate() {
        Point result = Utils.rotate(new Point(1, 0), new Point(0, 0), Math.toRadians(45));
        Point expectedResult = new Point(0.7071067811865475, 0.7071067811865475);
        assert(result.nearlyEquals(expectedResult)) : result;
    }

    private boolean approxEqual(double x, double y) {
        return (int)((x+0.000005)*100000) == (int)((y+0.000005)*100000);
    }

    private void assertApproxEqual(double ref, double value) {
        assert(approxEqual(ref, value));
    }

    private final double pi = Math.PI;

    @Test
    public void testCircleDiffMisc() {
        //// Page 8 (a1 > pi/2 and a1 < pi), case a2 > -pi/2 and a2 < 0
        // In both of these cases, the direction should be positive (CCW)
        assertApproxEqual(3.1, Utils.circleDiff(1.6584073464102091, -1.524777960769378));
        assertApproxEqual(-3.1, Utils.circleDiff(3.041592653589793, -0.058407346410208616));

        assertApproxEqual(0, Utils.circleDiff(0, 0));
        assertApproxEqual(pi/4, Utils.circleDiff(0, pi/4));
        assertApproxEqual(pi/2, Utils.circleDiff(0, pi/2));
        assertApproxEqual(3*pi/4, Utils.circleDiff(0, 3*pi/4));
        assertApproxEqual(pi, Utils.circleDiff(0, pi));

        assertApproxEqual(-pi/4, Utils.circleDiff(0, -pi/4));
        assertApproxEqual(-pi/2, Utils.circleDiff(0, -pi/2));
        assertApproxEqual(-3*pi/4, Utils.circleDiff(0, -3*pi/4));
        assertApproxEqual(-pi, Utils.circleDiff(0, -pi));

        // simplest case; counter-clockwise
        assertApproxEqual(1, Utils.circleDiff(0, 1));
        assertApproxEqual(3, Utils.circleDiff(0, 3));
        assertApproxEqual(pi, Utils.circleDiff(0, pi));

        // simple case; clockwise
        assertApproxEqual(-1, Utils.circleDiff(1, 0));
        assertApproxEqual(-3, Utils.circleDiff(3, 0));
        assertApproxEqual(-pi, Utils.circleDiff(pi, 0));

        // counter-clockwise, change in angle should be positive
        assertApproxEqual(0.08318530717958605, Utils.circleDiff(3.1, -3.1));

        // clockwise, change in angle should be negative
        assertApproxEqual(-0.08318530717958605, Utils.circleDiff(-3.1, 3.1));

        // complex case; should traverse counter-clockwise by pi/2 from 3.14159 to -1.5707
        assertApproxEqual(pi/2, Utils.circleDiff(pi, -pi/2));

        // clockwise from 0 to -1
        assertApproxEqual(-1, Utils.circleDiff(0, -1));

        // clockwise from 1 to -1
        assertApproxEqual(-2, Utils.circleDiff(1, -1));

        // complex case; should traverse clockwise by -3pi/4
        assertApproxEqual(-(3*pi/4), Utils.circleDiff(pi/2, -pi/4));

        assertApproxEqual(pi/2, Utils.circleDiff(pi - pi/4, pi + pi/4));
        assertApproxEqual(pi/4, Utils.circleDiff(pi, pi + pi/4));
        assertApproxEqual(pi/4, Utils.circleDiff(pi, -pi + pi/4));
    }

    @Test
    public void testCircleDiffRange() {
        // Fuzz the whole circle, asserting that |w| <= π; circleDiff should never return any value
        // outside the range [-π,π]
        double a1 = -pi;
        while (a1 < pi) {
            double a2 = -pi;
            while (a2 < pi) {
                double w = Utils.circleDiff(a1, a2);
                assert (Math.abs(w) <= pi);
                a2 += .1;
                a1 += .1;
            }
        }
    }

    @Test
    public void testCircleDiffDirectionalityCCW() {
        // Direction check; at each a1 from -pi to 0, do a half-circle
        // advance on a2 ranging [a1,a1+pi) (CCW) to verify direction.
        double a1 = -pi;
        while (a1 < pi) {
            double a2 = a1;
            while (a2 < a1 + pi) {
                // wrap a2 to keep it within [-pi,pi]
                double a2_ = a2;
                if (a2_ > pi) a2_ = -pi + (a2 - pi);
                if (a2_ < -pi) a2_ = pi - (a2 + pi);

                double w = Utils.circleDiff(a1, a2_);
                assert (Math.abs(w) <= pi) : a1 + " " + a2 + " " + w;
                assert (w >= 0);
                a2 += .1;
            }
            a1 += .1;
        }
    }

    @Test
    public void testCircleDiffDirectionalityCW() {
        // Direction check; at each a1 from [pi,-pi), do a half-circle
        // advance on a2 ranging [a1,a1-pi) (CW) to verify direction.
        double a1 = pi;
        while (a1 > -pi) {
            double a2 = a1;
            while (a2 > a1 - pi) {
                // wrap a2 to keep it within [-pi,pi]
                double a2_ = a2;
                if (a2_ > pi) a2_ = -pi + (a2 - pi);
                if (a2_ < -pi) a2_ = pi + (a2 + pi);

                double w = Utils.circleDiff(a1, a2_);
                assert (Math.abs(w) <= pi);
                assert (w <= 0);
                a2 -= .1;
            }
            a1 -= .1;
        }
    }
}
