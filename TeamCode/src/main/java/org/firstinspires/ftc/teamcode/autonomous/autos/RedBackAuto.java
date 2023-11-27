package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.autoParents.BackAutoParent;

// Import Custom Classes


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Back Auto", group = "Red Autos")

public class RedBackAuto extends OpMode {

    // Create Prop Location-Related Variables
    enum Locations {LEFT, FRONT, MIDDLE}
    private Locations propLocation;

    private BackAutoParent backAutoParent;

    public void init() {
        // Help the autoParent to initiate
        backAutoParent =  new BackAutoParent(hardwareMap, "RED");

        // Detect the team prop's location
        backAutoParent.detectProp(hardwareMap, telemetry, 5);

        // Generate all the RoadRunner Trajectory Sequences Ahead of Time
        backAutoParent.generateTrajectorySequences(telemetry);
    }

    public void init_loop() {

        }

    public void start() {
        backAutoParent.complexAuto(hardwareMap, telemetry);
    }

    public void loop() {

    }

}