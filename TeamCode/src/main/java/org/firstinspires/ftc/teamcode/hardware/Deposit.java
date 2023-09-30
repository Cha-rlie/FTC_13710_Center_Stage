package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Deposit extends SubsystemBase {

    // Declare variables here
    // Servos
    ServoEx GS; // Gripper Servo
    ServoEx ZA; // Zombie Axel Servo
    ServoEx V4B1; // Virtual Four Bar First Servo
    ServoEx V4B2; // Virtual Four Bar Second Servo

    // Slide Motors
    MotorEx DS1; // Deposit Slide 1
    MotorEx DS2; // Deposit Slide 2

    MotorGroup DS; // Deposit Slides

    // Initialise Safety Variables
    boolean gripperState = false; // Not Gripped

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);
        ZA = new SimpleServo(hardwareMap, "ZA", 0, 360, AngleUnit.DEGREES);
        V4B1 = new SimpleServo(hardwareMap, "V4B1", 0, 360, AngleUnit.DEGREES);
        V4B2 = new SimpleServo(hardwareMap, "V4B2", 0, 360, AngleUnit.DEGREES);

        DS1 = new MotorEx(hardwareMap, "DS1");
        DS2 = new MotorEx(hardwareMap, "DS2");
        DS2.setInverted(true);

        DS =  new MotorGroup(DS1, DS2);
        DS.setRunMode(Motor.RunMode.PositionControl);
        DS.setPositionCoefficient(0.05); //This is just what they had in the docs, but we are supposed to tune it
        DS.setPositionTolerance(20); // This the allowance for error we are giving the motors
        DS.resetEncoder();

    }

    public void grip() {
        if (!gripperState) {
            GS.rotateByAngle(40);
            gripperState = !gripperState;
        }
    }

    public void letGo() {
        if (gripperState) {
            GS.rotateByAngle(-40);
            gripperState = !gripperState;
        }
    }

    public void rotateGripperAngleToPickUp() {
        ZA.rotateByAngle(90);
    }

    public void rotateGripperAngleToDeposit()
    {
        ZA.rotateByAngle(-90);
    }

    public void rotateV4BToPickUp() {
        // Hopefully these move at the same time? I have put a low angle to not break it if not :)
        V4B1.rotateByAngle(10);
        V4B2.rotateByAngle(10);
    }

    public void rotateV4BToDeposit() {
        // Hopefully these move at the same time? I have put a low angle to not break it if not :)
        V4B1.rotateByAngle(10);
        V4B2.rotateByAngle(10);
    }

    public void powerSlides(int distanceToMove) { // Free-style
        DS.setTargetDistance(distanceToMove);

        while (!DS.atTargetPosition()) {
            DS.set(0.5);
        }

        DS.stopMotor();

    }

    public void slidesDown() {

        DS.setTargetPosition(0);

        DS.set(0);

        while (!DS.atTargetPosition()) {
            DS.set(0.5);
        }

        DS.stopMotor();
    }

    public void slidesUp() {
        DS.setTargetPosition(200);

        DS.set(0);

        while (!DS.atTargetPosition()) {
            DS.set(0.5);
        }

        DS.stopMotor();
    }

}
