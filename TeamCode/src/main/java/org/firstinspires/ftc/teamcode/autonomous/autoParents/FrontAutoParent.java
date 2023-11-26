package org.firstinspires.ftc.teamcode.autonomous.autoParents;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Import Vision Related Classes
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;

// Import RoadRunner Classes
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

// Import Custom Vision Classes
import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector;
import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector.Locations;
import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetection.AprilTagCameraDetector;

public class FrontAutoParent {

    // Declare Template-Related Variables
    String ALIANCE_COLOR;

    // Declare Prop Location-Related Variables
    propLocationDetector propLocationDetector;
    public Locations propLocation;
    WebcamName propCamera;

    // Create AprilTag-Related Variables
    AprilTagCameraDetector aprilTagDetector;
    private OpenCvCamera aprilTagCamera;
    private Locations aprilTagSide;
    private int[] tagsToSearchFor;

    // Declare Movement-Related Variables
    SampleMecanumDrive driveBase;
    int alianceAdjustor;
    double startX;
    double startY;
    int startHeading;
    TrajectorySequence chosenTeamPropScoringLocationTrajectory;
    TrajectoryVelocityConstraint halfSpeed;
    TrajectoryAccelerationConstraint halfAcceleration;

    // Declare RoadRunner Trajectory Sequences
    TrajectorySequence driveToLeftTeamPropScoringPosition;
    TrajectorySequence driveToFrontTeamPropScoringPosition;
    TrajectorySequence driveToRightTeamPropScoringPosition;
    TrajectorySequence driveBackToBackDrop;
    TrajectorySequence driveToNextAprilTag;
    TrajectorySequence driveToPixelStacksTrajectory;
    TrajectorySequence driveToBackdrop;
    TrajectorySequence driveToRightParkingLocationTrajectory;

    public FrontAutoParent(HardwareMap hardwareMap, String alianceColour) {
        ALIANCE_COLOR = alianceColour;

        if (ALIANCE_COLOR.equals("BLUE")) {
             tagsToSearchFor = new int[]{1, 2, 3};
             alianceAdjustor = 1;
        } else {
            tagsToSearchFor = new int[]{4, 5, 6};
            alianceAdjustor = -1;
        }


        // Assign and set-up the RoadRunner Mecanum Drive
        driveBase = new SampleMecanumDrive(hardwareMap);
        startX = -37;
        startY = 60* alianceAdjustor;
        startHeading = 90* alianceAdjustor;
        driveBase.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(startHeading)));
        halfSpeed = SampleMecanumDrive.getVelocityConstraint(45*0.8, Math.toRadians(130), 14.0011);
        halfAcceleration = SampleMecanumDrive.getAccelerationConstraint(68);

    }

    /**
     * Attempts to Detect the Prop's Location - Run in Init()
     *
     * @param timeToDetectFor Time that Detector has to Localise the Prop
     * @param hardwareMap OpMode's Hardwaremap
     * @param telemetry OpMode's Telemetry
     */
    public void detectProp (HardwareMap hardwareMap, Telemetry telemetry, int timeToDetectFor) {
        // Set up the TensorFlow Prop Detector
        propCamera = hardwareMap.get(WebcamName.class, "Webcam");
        propLocationDetector = new propLocationDetector(propCamera, ALIANCE_COLOR);

        // Detect the Team Prop's Location
        propLocation = propLocationDetector.detectPropLocation(telemetry, timeToDetectFor);

    }

    /**
     * Takes the time to create all the RoadRunner TrajectorySequences - Run in Init()
     *
     * @param  telemetry OpMode's Telemetry
     */
    public void generateTrajectorySequences (Telemetry telemetry) {
        telemetry.addLine("Generating TrajectorySequences Now...");
        telemetry.update();
        // TrajectorySequences to position the robot for scoring the purple pixel with the team prop
        driveToLeftTeamPropScoringPosition = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .lineTo(new Vector2d(startX, startY-(15*alianceAdjustor)))
                .turn(30*alianceAdjustor)
                .build();
        driveToFrontTeamPropScoringPosition = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .lineTo(new Vector2d(startX, startY-(25*alianceAdjustor)))
                .turn(180)
                .build();
        driveToRightTeamPropScoringPosition = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .lineTo(new Vector2d(startX, startY-(15*alianceAdjustor)))
                .turn(-30*alianceAdjustor)
                .build();

        driveToBackdrop = driveBase.trajectorySequenceBuilder(chosenTeamPropScoringLocationTrajectory.end())
                .splineToConstantHeading(new Vector2d(0+startX, (-33*alianceAdjustor)+startY), Math.toRadians(driveBase.getPoseEstimate().getHeading()))
                .turn(Math.toRadians(90*alianceAdjustor))
                .splineToConstantHeading(new Vector2d(0+startX, (-35*alianceAdjustor)+startY), Math.toRadians(180)) //Turns counter-clockwise
                .splineToConstantHeading(new Vector2d(41+startX, (-35*alianceAdjustor)+startY), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(65+startX,(-15*alianceAdjustor)+startY), Math.toRadians(180))
                //.lineTo(new Vector2d(0, -58))
                //.turn(Math.toRadians(90))
                //.lineTo(new Vector2d(83, -52))
                //.forward(90)
                .build();

        driveToNextAprilTag = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY()+(12*alianceAdjustor)))
                .build();

        driveToPixelStacksTrajectory = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(10+startX, (-33*alianceAdjustor)+startY), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20+startX, (-33*alianceAdjustor)+startY), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-43+startX, (-33*alianceAdjustor)+startY), Math.toRadians(180))
                .build();

        driveBackToBackDrop = driveBase.trajectorySequenceBuilder(driveToPixelStacksTrajectory.end())
                .splineToConstantHeading(new Vector2d(-20+startX, (-33*alianceAdjustor)+startY), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(10+startX, (-34*alianceAdjustor)+startY), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(35+startX, (-3*alianceAdjustor)+startY), Math.toRadians(180))
                //.lineToConstantHeading(new Vector2d(60+startX, -15+startY))
                .build();

        driveToRightParkingLocationTrajectory = driveBase.trajectorySequenceBuilder(driveBackToBackDrop.end())
                .lineToConstantHeading(new Vector2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY()+(-28*alianceAdjustor)))
                .back(17)
                //.splineToConstantHeading(new Vector2d(55+startX, -40+startY), Math.toRadians(180))
                .build();

    }

    public void positionRobotToScorePurplePixel() {
        switch(propLocation) {
            case LEFT:
            case NOT_FOUND:
                // Move to the correct scoring location and orientation
                chosenTeamPropScoringLocationTrajectory = driveToLeftTeamPropScoringPosition;
                break;
            case FRONT:
                // Move to the correct scoring location and orientation
                chosenTeamPropScoringLocationTrajectory = driveToFrontTeamPropScoringPosition;
                break;
            case RIGHT:
                // Move to the correct scoring location and orientation
                chosenTeamPropScoringLocationTrajectory = driveToRightTeamPropScoringPosition;
                break;
        }
        // Position the robot so that it is prepared to score the pixel
        driveBase.followTrajectorySequence(chosenTeamPropScoringLocationTrajectory);
    }

    /**
     * Moves Right Until Correct AprilTag Detected - Run in Auto()
     *
     * @param hardwareMap OpMode's HardwareMap
     * @param telemetry OpMode's Telemetry
     */
    public void moveToCorrectLocationWithAprilTags(HardwareMap hardwareMap, Telemetry telemetry) {
        // Set-up the AprilTag Detector
        aprilTagDetector = new AprilTagCameraDetector(telemetry,aprilTagCamera, tagsToSearchFor);

        // Start the detected AprilTagSide as NOT_FOUND
        aprilTagSide = Locations.NOT_FOUND;

        boolean keepSearching = true;
        int searchNumber = 1;

        // Keep searching if the AprilTag has not been found yet and stop after two tries
        // Note: If the AprilTag is not found, it will place the pixel on the first position, which is the last aprilTag it was moved to
        while (keepSearching && searchNumber < 3) {
            // Get the desired parking location as a Locations object
            aprilTagSide = aprilTagDetector.getDetectedSide(telemetry);
            // Check if the aprilTag location matches that of the team prop
            if (aprilTagSide == propLocation) {
                // If it does, stop searching and moving
                keepSearching = false;
            } else {
                // Move onto the next aprilTag
                driveBase.followTrajectorySequence(driveToNextAprilTag);
                searchNumber ++;
            }
        }

    }

    // TODO: Use the hardware Functions to code a scoring mechanism
    /**
     * Score the preloaded yellow pixel - Run in Auto()
     *
     * @param hardwareMap OpMode's HardwareMap
     */
    public void scorePreloadedYellowPixel(HardwareMap hardwareMap) {

    }

    /**
     * Complex Auto - Hypothetical 55 Points - Run in Auto()
     *
     * @param hardwareMap OpMode's HardwareMap
     */
    public void complexAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        // Place the purple pixel in the correct location
        positionRobotToScorePurplePixel();
        // TODO: Use Hardware functions to SCORE here
        // Drive to BackDrop
        driveBase.followTrajectorySequence(driveToBackdrop);
        // Drive to the correct AprilTag Location
        moveToCorrectLocationWithAprilTags(hardwareMap, telemetry);
        // Score the yellow pixel
        // TODO: Use Hardware functions to SCORE here
        // Drive to the rightmost pixel stack
        driveBase.followTrajectorySequence(driveToPixelStacksTrajectory);
        // Intake 2 pixels
        // TODO: Use Hardware functions to INTAKE here
        // Drive back to the backdrop
        driveBase.followTrajectorySequence(driveBackToBackDrop);
        // Score the pixels
        // TODO: Use Hardware functions to SCORE here
        // Drive and park in the parking location on the right of the backdrop
        driveBase.followTrajectorySequence(driveToRightParkingLocationTrajectory);

    }

}