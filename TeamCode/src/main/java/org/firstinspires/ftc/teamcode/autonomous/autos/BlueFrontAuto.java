package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector;

// Import Custom Classes
import org.firstinspires.ftc.teamcode.autonomous.autoParents.FrontAutoParent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Front Auto", group = "Blue Autos")

public class BlueFrontAuto extends OpMode {

    // Create Prop Location-Related Variables
    enum Locations {LEFT, FRONT, MIDDLE}
    private propLocationDetector.Locations propLocation;

    private FrontAutoParent frontAutoParent;

    public void init() {
        // Help the autoParent to initiate
        frontAutoParent = new FrontAutoParent(hardwareMap, "BLUE");

        // Detect the team prop's location
        frontAutoParent.detectProp(hardwareMap, telemetry, 5);

        // Generate all the RoadRunner Trajectory Sequences Ahead of Time
        frontAutoParent.generateTrajectorySequences(telemetry);
    }

    public void init_loop() {

    }

    public void start() {
        frontAutoParent.complexAuto(hardwareMap, telemetry);
    }

    public void loop() {

    }

}