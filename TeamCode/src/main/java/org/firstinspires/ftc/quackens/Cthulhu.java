package org.firstinspires.ftc.quackens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Cthulhu {
    private DcMotorEx[] motors;

    private Localizer localizer;

    public Cthulhu(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br,
                   Localizer localizer_)
    {
        motors = new DcMotorEx[]{fl, fr, bl, br};
        for (int i = 0; i < 4; ++i) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        localizer = localizer_;
    }

    public Pose2D getPose() { return localizer.getPose(); }

    private double[] directionMat = {
            -1, 1,
            -1, 1 };
    private double[] turnMat = {
            -1, 1,
            -1, 1 };
    private double[] strafeMat = {
            -1, -1,
             1,  1 };

    private double forwardPower = 0.0;
    private double strafePower = 0.0;
    private double turnPower = 0.0;

    public void forward(double p) { forwardPower = Utils.clamp(p, -1.0, 1.0); }
    public void strafe(double p) { strafePower = Utils.clamp(p, -1.0, 1.0); }
    public void turn(double p) { turnPower = Utils.clamp(p, -1.0, 1.0); }

    /*! Calculate and apply the motor inputs */
    public void update() {
        double[] motorPower = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; ++i)
            motorPower[i] = directionMat[i] * forwardPower
                    + (directionMat[i] * strafeMat[i] * strafePower)
                    + (directionMat[i] * turnMat[i] * turnPower);
        for (int i = 0; i < 4; ++i)
            motors[i].setPower(motorPower[i]);
    }
}
