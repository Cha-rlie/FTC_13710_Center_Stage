package org.firstinspires.ftc.teamcode.autonomous.propLocationDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class propLocationDetector {

    // Variables for vision processing
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static String TFOD_MODEL_FILE;

    public static final String[] LABELS = {"Prop"};

    // Variables for state management
    private String[] sides = new String[]{"LEFT","FRONT","RIGHT"};
    private String detectedSide;
    private Boolean currentlyDetecting;
    // Selectively Obscure Some Pixels (Essentially Cropping without Changing Size)
    private int[] leftDetectionClip = new int[]{0, 0, 500, 0}; // Left, Top, Right, Bottom
    private int[] frontDetectionClip = new int[]{300, 0, 300, 0};
    private int[] rightDetectionClip = new int[]{500, 0, 0, 0};

    private void initDetector(WebcamName webcam, String alianceColor) {
        if (alianceColor.equals("BLUE")) {
            TFOD_MODEL_FILE = "models/Blue_Team_Prop_Detector_File.tflite";
        } else {
            TFOD_MODEL_FILE = "models/Red_Team_Prop_Detector_File.tflite";
        }
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();
        // Only consider a prop detected if it is 75% confident or more
        tfod.setMinResultConfidence((float) 0.75);

        visionPortal = VisionPortal.easyCreateWithDefaults(webcam, tfod);
    }

    private String detectPropLocation() {
        //visionPortal.resumeStreaming();
        while (currentlyDetecting) {
            for (String side : sides) {
                // Selectively Obscure Some Pixels (Essentially Cropping without Changing Size)
                if (side.equals("LEFT")) {tfod.setClippingMargins(0, 0, 500, 0);}
                else if (side.equals("FRONT")) {tfod.setClippingMargins(300, 0, 300, 0);}
                else {tfod.setClippingMargins(500, 0, 0, 0);}
                List<Recognition> currentRecognitions = tfod.getRecognitions();

                if (currentRecognitions.size() == 1) {
                    // When the visionPortal is no longer needed, close to conserve resources
                    visionPortal.stopStreaming();
                    visionPortal.close();
                    detectedSide = side;
                    currentlyDetecting = false;
                    break;
                }
            }
        }
        return detectedSide;
    }

}
