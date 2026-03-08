package org.firstinspires.ftc.quackens;

public class PidController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    // current target value
    private double target = 0.0;

    // previous error
    private double e_prev = 0.0;

    // previous integral value
    private double Ti_prev = 0.0;

    // Limit the growth of the integral
    private double Ti_min = Double.MIN_VALUE;
    private double Ti_max = Double.MAX_VALUE;

    public PidController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setLimits(double Ti_min, double Ti_max) {
        this.Ti_min = Ti_min;
        this.Ti_max = Ti_max;
    }

    public void   setTarget(double target) { this.target = target; }
    public double getTarget() { return target; }

    public double compute(double dt, double currentReading) {
        return compute(dt, currentReading, 1.0);
    }

    public double compute(double dt, double currentReading, double dampingMultiplier) {
        double error = target - currentReading;
        return update(dt, error, dampingMultiplier);
    }

    public double update(double dt, double error, double dampingMultiplier) {
        // Calculate proportional term
        double Tp = Kp * error;

        // Calculate integrate term
        double Ti = Ti_prev + Ki * dt * 0.5 * (error + e_prev);
        Ti = Utils.clamp(Ti, Ti_min, Ti_max);

        double Td = Kd * (error - e_prev) * dampingMultiplier;

        Ti_prev = Ti;
        e_prev = error;

        return Tp + Ti + Td;
    }
}
