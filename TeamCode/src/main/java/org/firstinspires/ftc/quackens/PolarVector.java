package org.firstinspires.ftc.quackens;

public class PolarVector {
    /*! The vector's radial distance (or magnitude) */
    double r;

    /*! The vector's angle */
    double phi;

    PolarVector() { r = 1; phi = 0; }
    PolarVector(double r, double phi) {
        this.r = r;
        this.phi = phi;
    }

    /*! Convert to rectangular coordinate */
    Point toPoint() {
        return new Point(
                r * Math.cos(phi),
                r * Math.sin(phi));
    }
}
