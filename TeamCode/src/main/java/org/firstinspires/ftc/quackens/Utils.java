package org.firstinspires.ftc.quackens;

public class Utils {
    public static double clamp(double value, double min, double max) {
        if (value < min) return min;
        else return Math.min(value, max);
    }

    public static float clamp(float value, float min, float max) {
        if (value < min) return min;
        else return Math.min(value, max);
    }

    /*! Rotate point p around origin by theta radians
    // R = [cos(T) -sin(T) [x
    //      sin(T)  cos(T)] y]
     */
    public static Point rotate(Point p, Point origin, double theta) {
        final double x = p.x - origin.x;
        final double y = p.y - origin.y;
        final double xr = x * Math.cos(theta) - y * Math.sin(theta);
        final double yr = x * Math.sin(theta) - y * Math.cos(theta);
        return new Point(origin.x + xr, origin.y + yr);
    }

    /*! Calculate the distance between two angles on a circle [-π,π]
     *
     * Why?
     *   Calculating the shortest angular distance between two points on a circle requires
     * addressing a number of
     * edge cases. Consider the points  π/8 and -15/8π; they are very near on the circle, crossing
     * angle 0, but this is not mathematically obvious.
     */
    public static double circleDiff(final double t1, final double t2) {
        if (t2 > t1 && t2 > 0 && t1 < 0 && Math.abs(t2 - t1) > Math.PI)
            return -((Math.PI - t2) + (t1 + Math.PI));
        else if (t1 > t2 && t1 > 0 && t2 < 0 && Math.abs(t2 - t1) > Math.PI)
            return (Math.PI - t1) + (t2 + Math.PI);
        else
            return t2 - t1;
    }

    public static double circleDiff_(double t1, double t2) {
//        if (t1 < -Math.PI) t1 %= Math.PI;
//        else if (t1 > Math.PI) t1 %= -Math.PI;
//        if (t2 < -Math.PI) t2 %= Math.PI;
//        else if (t2 > Math.PI) t2 %= -Math.PI;

        // Alternative implementation suggested by perplexity.ai that does not pass the fuzzing
        // tests. It does call into question the above implementation for out of range inputs.
        // Requires more thought.
        return ((t2 - t1 + Math.PI) % (2 * Math.PI)) - Math.PI;
    }
}