package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetection.AprilTagCameraDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Front Auto", group = "Blue Autos")

public class BlueFrontAuto extends OpMode {

    // Create Prop Location-Related Variables
    enum Locations {LEFT, FRONT, MIDDLE}
    private Locations propLocation;

    // Create AprilTag-Related Variables
    AprilTagCameraDetection aprilTagDetector;
    private OpenCvCamera camera;
    private String aprilTagLocation;
    private int[] tagsToSearchFor = new int[]{1, 2, 3};

    public void init() {
        // Set-up the camera view
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // Set up the TensorFlow Prop Detector

        // Set-up the AprilTag Detector
        aprilTagDetector = new AprilTagCameraDetection(telemetry, camera, tagsToSearchFor);

        // Get the desired parking location as a string
        aprilTagLocation = aprilTagDetector.getDetectedSide(telemetry);
    }

    public void init_loop() {
        // Detect the corresponding location of the AprilTag
        // Continue searching until it is found
        while (aprilTagLocation == "NOT FOUND") {
            aprilTagLocation = aprilTagDetector.getDetectedSide(telemetry);
            telemetry.addData("AprilTag's Corresponding Location", aprilTagLocation);
            telemetry.update();
        }
    }

    public void start() {

    }

    public void loop() {

    }

}
