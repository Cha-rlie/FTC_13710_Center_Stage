package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetection.AprilTagCameraDetection;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "PARK Blue Front", group = "Blue Autos")

public class blueFrontPark extends OpMode {

    public ElapsedTime Timer = new ElapsedTime();

    Drivebase driveBase;
    IMU imu;
    GamepadEx driveOp;
    GamepadEx toolOp;
    SampleMecanumDrive drive;
    TrajectorySequence blueFrontPark;

    // Declaring Commands
    private Deposit deposit;
    private Intake intake;
    private Hang hang;
    private DroneLauncher shooter;

    public void init() {
        driveBase = new Drivebase(hardwareMap);
        // Initialise the imuGyro with the correct orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        hang = new Hang(hardwareMap);
        shooter = new DroneLauncher(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence blueFrontPark = drive.trajectorySequenceBuilder(new Pose2d(-36.70, 62.80, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-39.33, 32.57), Math.toRadians(270.00))
                .splineTo(new Vector2d(62.99, 11.17), Math.toRadians(355.30))
                .build();


    }

    public void start() {
        drive.followTrajectorySequence(blueFrontPark);

        intake.Rspin();
    }

    public void loop() {

    }

}
