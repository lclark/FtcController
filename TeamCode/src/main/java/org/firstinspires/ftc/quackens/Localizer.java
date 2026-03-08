package org.firstinspires.ftc.quackens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Localizer {
    public final double TicksPerMM = 2048 / 150.79644737231007;

    // mm location of longitudinal deadwheel relative to robot center of rotation.
    private final Point longDwLocation = new Point(107.5, 0);

    // mm location of lateral deadwheel relative to robot center of rotation.
    private final Point lateralDwLocation = new Point(0, -145);

    final private IMU imu;
    final private DcMotorEx longDw;
    final private DcMotorEx lateralDw;

    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double turnRate = 0.0;

    private int prevXTicks = 0;
    private int prevYTicks = 0;
    private double prevHeading = 0.0;

    Localizer(IMU imu_, DcMotorEx longitudinalDeadWheel, DcMotorEx lateralDeadWheel) {
        imu = imu_;
        longDw = longitudinalDeadWheel;
        lateralDw = lateralDeadWheel;

        longDw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralDw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*! @return current position and heading */
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, velocityX, velocityY, AngleUnit.RADIANS, turnRate);
    }

    public void reset() {
        imu.resetYaw();
        prevXTicks = lateralDw.getCurrentPosition();
        prevYTicks = longDw.getCurrentPosition();
    }

    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        heading = orientation.getYaw(AngleUnit.RADIANS);

        final int xTicks = lateralDw.getCurrentPosition();
        final int yTicks = longDw.getCurrentPosition();

        final int deltaXTicks = prevXTicks - xTicks;
        final int deltaYTicks = prevYTicks - yTicks;
        final double deltaHdg = Utils.circleDiff(heading, prevHeading);

        prevXTicks = xTicks;
        prevYTicks = yTicks;
        prevHeading = heading;

        // Calculate the x,y correction needed based on the change in heading and deadwheel locations.
        // As the robot rotates around its center, the deadwheels will move, but the robot's
        // location will not. We need to calculate the deadwheel's expected change resulting from
        // the change in heading and subtract that from the deadwheels measured distance.
        final double sX = -deltaHdg * lateralDwLocation.hypot();
        final double sY = -deltaHdg * longDwLocation.hypot();

        // we now have the measured change from the encoders and the corrections to account for how
        // the change in heading caused the deadwheels to move. This is the x,y displacement of the
        // robot within its frame. We need to rotate those values around the robot's center to
        // convert them into the Global frame.
        final Point delta = Utils.rotate(
                new Point((-deltaXTicks / TicksPerMM) + sX,
                        (-deltaYTicks / TicksPerMM) + sY),
                new Point(0,0),
                heading);

        x += delta.x;
        y += delta.y;

        // velocityX = delta.x / dt;
        turnRate = angularVelocity.zRotationRate;
    }
}
