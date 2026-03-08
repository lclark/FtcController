package org.firstinspires.ftc.quackens;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Dashboard: Test Features")
public class DashboardTest extends OpMode {
    Cthulhu robot;
    Localizer localizer;
    PositionController controller;
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0.0;

    @Override             //Override is used for each loop
    public void init() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ));
        imu.initialize(imuParams);

        localizer = new Localizer(imu,
                hardwareMap.get(DcMotorEx.class, "fl"),
                hardwareMap.get(DcMotorEx.class, "fr") );

        robot = new Cthulhu(
                hardwareMap.get(DcMotorEx.class, "fl"),
                hardwareMap.get(DcMotorEx.class, "fr"),
                hardwareMap.get(DcMotorEx.class, "bl"),
                hardwareMap.get(DcMotorEx.class, "br"),
                localizer);

        controller = new PositionController(robot);
    }

    @Override
    public void init_loop() {
        //Happens repeatedly during init
        localizer.reset();
        localizer.update();
        Pose2D pose = localizer.getPose();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("x", pose.getX(DistanceUnit.MM));
        packet.put("y", pose.getY(DistanceUnit.MM));
        packet.put("heading", pose.getHeading(AngleUnit.DEGREES));

        packet.put("fwd", controller.fwdSignal);
        packet.put("stf", controller.strafeSignal);
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = timer.seconds();

        localizer.update();
        controller.setTarget(localizer.getPose());
    }

    @Override
    public void loop() {
        final double currentTime = timer.seconds();
        final double dt = currentTime - lastTime; // deltaT, change in time; time elapsed since last loop iteration
        lastTime = currentTime;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // read latest values from navigation sensors and integrate position
        localizer.update();

        Pose2D pose = localizer.getPose();

        packet.put("x", pose.getX(DistanceUnit.MM));
        packet.put("y", pose.getY(DistanceUnit.MM));
        packet.put("heading", pose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("x", pose.getX(DistanceUnit.MM));
        telemetry.addData("y", pose.getY(DistanceUnit.MM));
        telemetry.addData("heading", pose.getHeading(AngleUnit.DEGREES));

        // Calculate control signals based
        controller.update(dt);

        packet.put("fwd", controller.fwdSignal);
        packet.put("stf", controller.strafeSignal);

        // Apply all control settings to the hardware.
        robot.update();

        //Happens repeatedly during the program
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        //Happens once after stop
        robot.forward(0.0);
    }
}
