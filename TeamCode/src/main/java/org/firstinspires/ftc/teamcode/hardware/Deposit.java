package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Deposit extends SubsystemBase {

    // Servos
    ServoEx Gripper; // Gripper
    public ServoEx Wrist; // Wrist
    public ServoEx V4B1; // V4B
    public ServoEx V4B2;

    // Slide Motors
    public MotorEx DS1; // Deposit Slide 1
    public MotorEx DS2; // Deposit Slide 2

    public MotorGroup DS; // Deposit Slides
    // Slide positions
    public int min = 0;
    public int max = 1500;
    public double power = 1;
    public double wristPos = 90;

    // Initialise Safety Variables
    public boolean gripperState = false; // Not Gripped

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);


        DS1 = new MotorEx(hardwareMap, "DS1");
        DS2 = new MotorEx(hardwareMap, "DS2");

        int MIN_ANGLE = 0;
        int MAX_ANGLE = 180;

        V4B1 = new SimpleServo(hardwareMap, "V4B1", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        V4B2 = new SimpleServo(hardwareMap, "V4B2", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        Wrist = new SimpleServo(hardwareMap, "W", 0, 300, AngleUnit.DEGREES);
        Gripper = new SimpleServo(hardwareMap, "G", 0, 300, AngleUnit.DEGREES);

        V4B1.setInverted(true);

        V4B1.turnToAngle(180);
        V4B2.turnToAngle(180);
        Gripper.turnToAngle(60);
        Wrist.turnToAngle(180);

        Timer timer = new Timer(2);
        timer.start();

        while(!timer.done()) {
        }
//
//        DS =  new MotorGroup(DS1, DS2);
//        DS.setRunMode(Motor.RunMode.PositionControl);

        resetPosition();
    }

    public void grip() {
        if (gripperState) {
            Gripper.turnToAngle(0);
            //gripperState = !gripperState;
        } else {
            Gripper.turnToAngle(120);
        }
    }

    public void placing() {
        V4B1.turnToAngle(180);
        V4B2.turnToAngle(180);
        wristPos = 180;
    }

    public void manualV4BControl(double angle, Telemetry telemetry) {
        int scaling = 10;

        V4B1.rotateByAngle(-angle  * scaling);
        V4B2.rotateByAngle(-angle  * scaling);

        telemetry.addData("V4B: ", V4B1.getAngle());
    }


    public void rotateWristToAngle(double angle) {
        // Pretty much, the wrist is a temperamental thing. Its max and min values change depending
        // on where the V4B, and if those limits aren't adhered to, the belt will skip badly. To move
        // the wrist, this method clamps values according to the V4B position, so it won't break anything.

        double m = 0.65625;
        double c = 10;
        double centerWristPos = m * V4B1.getAngle() + c;

        double min = centerWristPos;
        double max = centerWristPos;

        Wrist.turnToAngle(MathUtils.clamp(angle, min, max));
    }


    public void resetPosition() {
//        DS.setRunMode(Motor.RunMode.RawPower);
//
//        while(DS1.motorEx.getCurrent(CurrentUnit.AMPS) < 1.5) {
//            DS.set(0.3);
//        }

//        DS.resetEncoder();
        DS1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DS2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void upSlides() {
        DS1.motor.setTargetPosition(max);
        DS2.motor.setTargetPosition(max);

        DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DS1.motor.setPower(power);
        DS2.motor.setPower(power);

        placing();
    }

    public void downSlides() {
        DS1.motor.setTargetPosition(min);
        DS2.motor.setTargetPosition(min);

        DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DS1.motor.setPower(power);
        DS2.motor.setPower(power);
    }

    public void powerOffSlides() {
        DS1.motor.setPower(0);
        DS2.motor.setPower(0);
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

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        if((currentPos < wantedPos + range) && currentPos > wantedPos - range) {
            return true;
        } else {
            return false;
        }
    }
}
