package org.firstinspires.ftc.quackens;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Translate {
    public static Point computeVector(final Point from, final Point to) {
        return new Point(to.x - from.x, to.y - from.y);
    }

    public static Point computeDisplacement(final Pose2D from, final Pose2D to) {
        // returns x,y change, but this is in the world coordinate frame and needs to be converted
        // to the robot's frame
        final Point vW = computeVector(new Point(from), new Point(to));

        // what is the angle of this vector?
        final double vT = Math.atan2(vW.y, vW.x);

        // and the magnitude?
        final double vR = vW.hypot();

        // Using the current heading, compute the forward and strafe components
        final double hT = from.getHeading(AngleUnit.RADIANS); // current heading
        final double rT = hT - vT;                            // relative heading

        final double yDisplacement = vR * Math.sin(rT);
        final double xDisplacement = vR * Math.cos(rT);

        return new Point(xDisplacement, yDisplacement);
    }
}
