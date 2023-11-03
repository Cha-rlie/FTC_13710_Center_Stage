package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Back Auto", group = "Blue Autos")

public class BlueBackAuto extends OpMode {

    // Create Prop Location-Related Variables
    enum Locations {LEFT, FRONT, MIDDLE}
    private Locations propLocation;

    // Create AprilTag-Related Variables
    AprilTagCameraDetection aprilTagDetector;
    private OpenCvCamera camera;
    private String aprilTagLocation;
    private int[] tagsToSearchFor = new int[]{1, 2, 3};
    public ElapsedTime Timer = new ElapsedTime();

    Drivebase driveBase;
    IMU imu;
    GamepadEx driveOp;
    GamepadEx toolOp;
    SampleMecanumDrive drive;
    TrajectorySequence autoJustParkMiddle;

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

        driveBase.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBase.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBase.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBase.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBase.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Complete transfer command

        deposit.safeTimer.reset();

        boolean commandFinished = false;

        while(!commandFinished) {
            deposit.Wrist.turnToAngle(deposit.defaultWrist);
            if(deposit.safeTimer.seconds() > 0 && deposit.safeTimer.seconds() < 0.5) {
                deposit.V4B1.turnToAngle(deposit.rampPosition);
                deposit.V4B2.turnToAngle(deposit.rampPosition);
            } else if(deposit.safeTimer.seconds() > 0.5 && deposit.safeTimer.seconds() < 1) {
                deposit.Gripper.turnToAngle(deposit.closedPosition);
            } else if(deposit.safeTimer.seconds() > 1 && deposit.safeTimer.seconds() < 1.5) {
                deposit.safe();
            } else if(deposit.safeTimer.seconds() > 1.5) {
                deposit.visible();
                commandFinished = true;
            }
        }

//        // Set-up the camera view
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//
//        aprilTagDetector = new AprilTagCameraDetection(telemetry, camera, tagsToSearchFor);
//
//        // Get the desired parking location as a string
//        aprilTagLocation = aprilTagDetector.getDetectedSide(telemetry);


    }

    public void start() {
        int tileLength = 61;

        telemetry.addLine("start has run");

//        driveBase.drive(100, 0, 0.5, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        driveBase.backwards(-tileLength, 0, -0.8, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Timer.reset();
        while(Timer.seconds() < 3) {
            driveBase.rotate(90, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1);
        }

        driveBase.resetEncoders();
        driveBase.backwards(-tileLength, 90, -0.8, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while(Timer.seconds() < 3) {
            driveBase.rotate(90, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1);
        }

        deposit.place();

        telemetry.update();

    }

    public void loop() {
        telemetry.addData("Distance Travelled: ", driveBase.getDistanceTravelled());
        telemetry.update();

    }

}
