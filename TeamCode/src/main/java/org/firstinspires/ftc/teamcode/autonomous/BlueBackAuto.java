package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetection.AprilTagCameraDetection;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

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
    public ElapsedTime Timerrrr = new ElapsedTime();

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

        // Complete transfer command

//        deposit.safeTimer.reset();
//
//        boolean commandFinished = false;
//
//        while(!commandFinished) {
//            deposit.Wrist.turnToAngle(deposit.defaultWrist);
//            if(deposit.safeTimer.seconds() > 0 && deposit.safeTimer.seconds() < 1) {
//                deposit.V4B1.turnToAngle(deposit.rampPosition);
//                deposit.V4B2.turnToAngle(deposit.rampPosition);
//            } else if(deposit.safeTimer.seconds() > 1 && deposit.safeTimer.seconds() < 2) {
//                deposit.Gripper.turnToAngle(80);
//            } else if(deposit.safeTimer.seconds() > 2 && deposit.safeTimer.seconds() < 3) {
//                deposit.safe();
//                commandFinished = true;
//            }
//        }

//        // Set-up the camera view
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//
//        aprilTagDetector = new AprilTagCameraDetection(telemetry, camera, tagsToSearchFor);
//
//        // Get the desired parking location as a string
//        aprilTagLocation = aprilTagDetector.getDetectedSide(telemetry);

        // RoadRunner Trajectories



//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
//        drive.setPoseEstimate(startPos);
//
//        autoJustParkMiddle = drive.trajectorySequenceBuilder(startPos)
//                .forward(20)
//                .build();


    }

    public void init_loop() {
        // Detect the corresponding location of the AprilTag
        // Continue searching until it is found
//        while (aprilTagLocation == "NOT FOUND") {
//            aprilTagLocation = aprilTagDetector.getDetectedSide(telemetry);
//            telemetry.addData("AprilTag's Corresponding Location", aprilTagLocation);
//            telemetry.update();
//        }
    }

    public void start() {
        Timerrrr.reset();

//
//        wait(2000);
//
//        driveBase.frontLeft.setPower(0);
//        driveBase.frontRight.setPower(0);
//        driveBase.rearLeft.setPower(0);
//        driveBase.rearRight.setPower(0);

//        drive.followTrajectorySequence(autoJustParkMiddle);

//        deposit.V4B1.turnToAngle(290);
//        deposit.V4B2.turnToAngle(290);
//        deposit.Wrist.turnToAngle(40);
//
//        while(deposit.DS1.motor.getCurrentPosition() < deposit.max-20) {
//            deposit.DS1.motor.setTargetPosition(deposit.max);
//            deposit.DS2.motor.setTargetPosition(deposit.max);
//
//            deposit.DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            deposit.DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            deposit.DS1.motor.setPower(deposit.power);
//            deposit.DS2.motor.setPower(deposit.power);
//        }

    }

    public void loop() {

        driveBase.frontLeft.setPower(0.3);
        driveBase.frontRight.setPower(0.3);
        driveBase.rearLeft.setPower(0.3);
        driveBase.rearRight.setPower(0.3);


//        driveBase.frontLeft.setTargetPosition(-580);
//        driveBase.frontRight.setTargetPosition(580);
//        driveBase.rearLeft.setTargetPosition(580);
//        driveBase.rearRight.setTargetPosition(-580);



    }

}
