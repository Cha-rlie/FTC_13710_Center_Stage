package org.firstinspires.ftc.teamcode.autonomous.propLocationDetection;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class propLocationDetector {

    // Variables for vision processing
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static String TFOD_MODEL_ASSET;
    public static final String[] LABELS = {"Prop"};
    private Timing.Timer detectionTimer;

    // Variables for state management
    public enum Locations {LEFT, FRONT, RIGHT, NOT_FOUND}
    private Locations detectedSide = Locations.NOT_FOUND;
    private Boolean currentlyDetecting = true;

    public propLocationDetector(WebcamName webcam, String alianceColor) {
        if (alianceColor.equals("BLUE")) {
            TFOD_MODEL_ASSET = "Blue_Team_Prop_Detector_Model.tflite";
        } else {
            TFOD_MODEL_ASSET = "Red_Team_Prop_Detector_Model.tflite";
        }
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        // Only consider a prop detected if it is 75% confident or more
        tfod.setMinResultConfidence((float) 0.75);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCamera(BuiltinCameraDirection.BACK)
                .enableLiveView(true)
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessor(tfod)
                .build();
    }

    public Locations detectPropLocation(Telemetry telemetry, int timeToDetectFor) {
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
        detectionTimer = new Timing.Timer(timeToDetectFor, TimeUnit.SECONDS);
        detectionTimer.start();
        while (!detectionTimer.done() && currentlyDetecting) {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Current Recognitions", currentRecognitions);
            telemetry.addData("# of Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            if (currentRecognitions.size() == 1) {
                // Stop trying to detect the object once a detection has occurec
                currentlyDetecting = false;

                // Create a local, temporary variable to hold the recognition
                Recognition recognition = currentRecognitions.get(0);

                // Get the x and y coordinates of the recognition
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                // Check which third of the camera frame the recognition was in
                // From this, estimate the prop location
                if (x < 430) {
                    detectedSide = Locations.LEFT;
                } else if (x > 850) {
                    detectedSide = Locations.RIGHT;
                } else {
                    detectedSide = Locations.FRONT;
                }

                telemetry.addData("Time Remaining", detectionTimer.remainingTime());
                telemetry.addData("Prop Location", detectedSide);
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }
            telemetry.update();
        }
        return detectedSide;
    }
//        while (currentlyDetecting && !detectionTimer.done()) {
//            for (Locations side : sides) {
//                telemetry.addData("Time Remaining", detectionTimer.remainingTime());
//                // Selectively Obscure Some Pixels (Essentially Cropping without Changing Size)
//                if (side == Locations.LEFT) {tfod.setClippingMargins(0, 0, 450, 0);}
//                else if (side == Locations.FRONT) {tfod.setClippingMargins(200, 0, 200, 0);}
//                else {tfod.setClippingMargins(450, 0, 0, 0);}
//                List<Recognition> currentRecognitions = tfod.getRecognitions();
//                telemetry.addData("CurrentRecognitions", currentRecognitions);
//                telemetry.update();
//                if (currentRecognitions.size() == 1) {
//                    // When the visionPortal is no longer needed, close to conserve resources
//                    visionPortal.stopStreaming();
//                    visionPortal.close();
//                    detectedSide = side;
//                    currentlyDetecting = false;
//                }
//            }
//        }
//        telemetry.addData("Detection Done", detectedSide);
//        telemetry.update();
//        return detectedSide;

}
