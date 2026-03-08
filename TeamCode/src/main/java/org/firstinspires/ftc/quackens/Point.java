package org.firstinspires.ftc.quackens;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Point {
    public final double x;
    public final double y;

    Point() {
        x = 0;
        y = 0;
    }

    Point(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    // Assumes units MM
    Point(final Pose2D pose) {
        this.x = pose.getX(DistanceUnit.MM);
        this.y = pose.getY(DistanceUnit.MM);
    }

    public double hypot() {
        return Math.sqrt((x*x) + (y*y));
    }

    public boolean equals(Point p) {
        return p.x == x && p.y == y;
    }

    public boolean nearlyEquals(Point p) {
        return (int)((x+0.000005)*100000) == (int)((p.x+0.000005)*100000)
                && (int)((y+0.000005)*100000) == (int)((p.y+0.000005)*100000);
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
