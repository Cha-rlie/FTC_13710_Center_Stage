package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Deposit extends SubsystemBase {

    // Servos
    ServoEx Gripper; // Gripper
    ServoEx Wrist; // Wrist
    public ServoEx V4B1; // V4B
    public ServoEx V4B2;

    // Slide Motors
    public MotorEx DS1; // Deposit Slide 1
    public MotorEx DS2; // Deposit Slide 2

    public MotorGroup DS; // Deposit Slides
    // Slide positions
    int min = 0;
    int transfer = 200;
    int max = 1000;
    int tolerance =  50;


    // Initialise Safety Variables
    boolean gripperState = false; // Not Gripped

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);


        DS1 = new MotorEx(hardwareMap, "DS1");
        DS2 = new MotorEx(hardwareMap, "DS2");
        DS2.setInverted(true);

        int MIN_ANGLE = 0;
        int MAX_ANGLE = 180;

        V4B1 = new SimpleServo(hardwareMap, "V4B1", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        V4B2 = new SimpleServo(hardwareMap, "V4B2", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        Wrist = new SimpleServo(hardwareMap, "W", 0, 300, AngleUnit.DEGREES);

        V4B1.setInverted(true);

        V4B1.turnToAngle(90);
        V4B2.turnToAngle(90);
        Wrist.turnToAngle(0);

        Timer timer = new Timer(2);
        timer.start();

        while(!timer.done()) {
        }

        DS =  new MotorGroup(DS1, DS2);
        DS.setRunMode(Motor.RunMode.PositionControl);

        resetPosition();
    }

    public void grip() {
        if (!gripperState) {
            Gripper.rotateByAngle(40);
            gripperState = !gripperState;
        }
    }

    public void letGo() {
        if (gripperState) {
            Gripper.rotateByAngle(-40);
            gripperState = !gripperState;
        }
    }

    public void pickUp() {
        V4B1.turnToAngle(6.4);
        V4B2.turnToAngle(6.4);
    }

    public void placing() {
        V4B1.turnToAngle(180);
        V4B2.turnToAngle(180);
    }

    public void manualV4BControl(double angle, Telemetry telemetry) {
        int scaling = 10;

        V4B1.rotateByAngle(-angle  * scaling);
        V4B2.rotateByAngle(-angle  * scaling);

        telemetry.addData("V4B: ", V4B1.getAngle());
    }

    public void resetPosition() {
//        DS.setRunMode(Motor.RunMode.RawPower);
//
//        while(DS1.motorEx.getCurrent(CurrentUnit.AMPS) < 1.5) {
//            DS.set(0.3);
//        }

        DS.resetEncoder();
        DS1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DS2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DS.setRunMode(Motor.RunMode.PositionControl);
        DS.stopMotor();

        // set and get the position coefficient
        DS.setPositionCoefficient(0.1);
        double kP = DS.getPositionCoefficient();
        DS.setPositionTolerance(30);   // allowed maximum error
    }

    public void upSlides() {
        // set the target position
        DS1.setTargetPosition(max);      // an integer representing
        // desired tick count
        DS.set(0);

        if (!DS1.atTargetPosition()) {
            DS.set(-1);
        }
    }

    public void downSlides() {
        // set the target position
        DS1.setTargetPosition(min);      // an integer representing
        // desired tick count
        DS.set(0);

        if (!DS1.atTargetPosition()) {
            DS.set(1);
        }
    }

    public void moveSlides(Telemetry telemetry) {
        // set the target position
        DS1.setTargetPosition(max);      // an integer representing
        // desired tick count
        DS.set(0);

        DS.setPositionCoefficient(0.1);
        double kP = DS.getPositionCoefficient();
        DS.setPositionTolerance(30);   // allowed maximum error

        // perform the control loop
        while (!DS1.atTargetPosition()) {
            DS.set(-1);
        }

        DS.stopMotor(); // stop the motor
    }
}
