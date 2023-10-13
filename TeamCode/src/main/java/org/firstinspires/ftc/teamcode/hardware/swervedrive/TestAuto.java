package org.firstinspires.ftc.teamcode.hardware.swervedrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoGoesBrrr", group = "Linear Opmode")

public class TestAuto extends LinearOpMode {
    Robot robot;
    int tileLength = 61;  // in cm
    int robotLength = 45; // in cm

    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();


        robot.driveController.rotateModules(Vector2d.FORWARD, true, 4000, this);
        robot.driveController.drive(Vector2d.FORWARD, tileLength+20, 1, this);

        //rotate modules to face to the right
        robot.driveController.rotateModules(Vector2d.RIGHT, true, 4000, this);
        robot.driveController.drive(Vector2d.RIGHT, tileLength*3, 1, this);

        //turn to face robot backwards
        robot.driveController.rotateRobot(Angle.BACKWARD, this);


    }
}

