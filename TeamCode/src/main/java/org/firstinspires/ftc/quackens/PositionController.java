package org.firstinspires.ftc.quackens;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class PositionController {
    // needs a robot to control... The robot interface is
    /*
    double getTurnRate() radians/second
    double getHeading() radians
    void turn(double [-1.0, 1.0])    // set the turn input
    Point getVelocity()  the robot's velocity in mm/s, x and y

     */
    public static double HDG_KP = 1.0;

    public static double FWD_KP = 0.002;
    public static double FWD_KI = 0.001;
    public static double FWD_KD = 0.005;

    public static double STF_KP = 0.002;
    public static double STF_KI = 0.001;
    public static double STF_KD = 0.005;

    Cthulhu robot;

    public static double targetX = 0.0;
    public static double targetY = 0.0;
    public static double targetHeading = 0.0;

    private PidController hdgPid;
    private PidController fwdPid;
    private PidController strafePid;

    public double fwdSignal;
    public double turnSignal;
    public double strafeSignal;

    PositionController(Cthulhu robot) {
        this.robot = robot;

        // The heading error is radians, a very small value. Initial Kp of 1.0 is good
        hdgPid = new PidController(HDG_KP, 0.0, 0.0);
        hdgPid.setLimits(-5.0, 5.0);

        // forward and strafe errors are in mm; be careful with large Kp values.
        fwdPid = new PidController(FWD_KP, FWD_KI, FWD_KD);
        fwdPid.setLimits(-FWD_KI*4, FWD_KD*4);

        strafePid = new PidController(STF_KP, STF_KI, STF_KD);
        strafePid.setLimits(-STF_KD*4, STF_KD*4);

    }

    public void setTarget(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetHeading = heading;
    }

    public void setTarget(final Pose2D pose) {
        setTarget(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM), pose.getHeading(AngleUnit.RADIANS));
    }

    public void update(double dt) {
        // Calculate required heading change
        Pose2D pose = robot.getPose();
        final double error = Utils.circleDiff(pose.getHeading(AngleUnit.RADIANS), targetHeading);
        final double turnSignal = Utils.clamp(hdgPid.update(dt, error, 1.0), -1.0, 1.0);
        robot.turn(turnSignal);

        // Calculate position change; compute the longitudinal and lateral components of the position displacement (error)
        final Point displacement = Translate.computeDisplacement(pose,
                new Pose2D(DistanceUnit.MM, targetX, targetY, AngleUnit.RADIANS, targetHeading));
        final double d = Math.abs(displacement.x) + Math.abs(displacement.y);
        if (d != 0) {
            fwdSignal = Utils.clamp(
                    fwdPid.update(dt, -displacement.y, 1.0),
                    -1.0, 1.0);
///  XXX DISPLACEMENT IS PROBABLY TECHNICALLY INCORRECT
            strafeSignal = Utils.clamp(
                    strafePid.update(dt, displacement.x, 1.0),
                    -1.0, 1.0);

            robot.forward(fwdSignal);
            robot.strafe(strafeSignal);
        }
    }
}
