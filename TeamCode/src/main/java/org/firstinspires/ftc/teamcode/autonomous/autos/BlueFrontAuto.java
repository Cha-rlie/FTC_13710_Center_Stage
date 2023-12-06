package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.autoParents.FrontAutoParentTest;
import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector;

// Import Custom Classes
import org.firstinspires.ftc.teamcode.autonomous.autoParents.FrontAutoParent;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.PlaceCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Front Auto", group = "Blue Autos")

public class BlueFrontAuto extends OpModeTemplate {

    // Create Prop Location-Related Variables
    enum Locations {LEFT, FRONT, MIDDLE}
    private propLocationDetector.Locations propLocation;

    private FrontAutoParent frontAutoParent;

    private SampleMecanumDrive drive;
    private TrajectorySequence positionRobotforLeft;
    private TrajectorySequence positionRobotforFront;
    private TrajectorySequence positionRobotforRight;

    private boolean autoAlreadyRun;

    public void initialize() {
        initHardware(false);
        // Help the autoParent to initiate
        frontAutoParent = new FrontAutoParent(hardwareMap, "BLUE");

        // Detect the team prop's location
        propLocation = frontAutoParent.detectProp(hardwareMap, telemetry, 10);

        drive = new SampleMecanumDrive(hardwareMap);

        // Generate all the RoadRunner Trajectory Sequences Ahead of Time
//        frontAutoParent.generateTrajectorySequences(telemetry);
//        positionRobotforLeft = drive.trajectorySequenceBuilder()
//        positionRobotforFront = drive.trajectorySequenceBuilder()
//        positionRobotforRight = drive.trajectorySequenceBuilder()
//
//        TrajectorySequence driveToBackBoard = drive.trajectorySequenceBuilder(new Pose2d(-36.62, 67.62, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(-42.71, 44.23))
//                .build();
//        drive.setPoseEstimate(driveToBackBoard.start());

        deposit.V4B.turnToAngle(180);

        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while(wait.seconds() < 3) {
        }

        deposit.Gripper.turnToAngle(110);

        wait.reset();
        while(wait.seconds() < 3) {
        }


        deposit.V4B.turnToAngle(270);
    }



    public void run() {
        if (!autoAlreadyRun) {
            // Stop it from still trying to detect
            frontAutoParent.propLocationDetector.currentlyDetecting = false;

            // Show the detected side to the telemetry
            telemetry.addData("DetectedSide", propLocation);
            telemetry.update();

            Trajectory untiled = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .back(50)
                    .build();
            drive.setPoseEstimate(untiled.start());
            drive.followTrajectory(untiled);

            // Generate the autos
            TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.62, 67.62, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(-42.71, 44.23))
                    .build();

            TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(-42.71, 44.23, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(-33.96, 60.59))
                    .lineTo(new Vector2d(-33.96, 14))
                    .build();

            TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(-33.96, 14.00, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(14.00, 14.00))
                    .lineToLinearHeading(new Pose2d(41.18, 30.72, Math.toRadians(180.00)))
                    .build();

            drive.setPoseEstimate(untitled0.start());

            switch(propLocation) {
                case LEFT:
                    //do thing
                case FRONT:
                    //do thing
                case RIGHT:
                    //do thing
            }

            // Run the trajectory sequences
//            drive.followTrajectorySequence(untitled0);
//            drive.followTrajectorySequence(untitled1);
//            drive.followTrajectorySequence(untitled2);
        }

//        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.24, 61.92, Math.toRadians(90.00)))
//                .lineToLinearHeading(new Pose2d(-37.76, 10.94, Math.toRadians(180.00)))
//                .lineTo(new Vector2d(30.91, 11.32))
//                .lineToLinearHeading(new Pose2d(43.85, 34.53, Math.toRadians(180.00)))
//                .build();
//        drive.setPoseEstimate(untitled0.start());

//        frontAutoParent.complexAuto(hardwareMap, telemetry);
    }


}