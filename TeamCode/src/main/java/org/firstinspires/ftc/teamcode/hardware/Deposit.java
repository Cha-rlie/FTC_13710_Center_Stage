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
    public double defaultWrist = 140;

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);

        DS1 = new MotorEx(hardwareMap, "DS1", Motor.GoBILDA.RPM_1150);
        DS2 = new MotorEx(hardwareMap, "DS2", Motor.GoBILDA.RPM_1150);

        int MIN_ANGLE = 0;
        int MAX_ANGLE = 355;

        V4B1 = new SimpleServo(hardwareMap, "V4B1", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        V4B2 = new SimpleServo(hardwareMap, "V4B2", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        Wrist = new SimpleServo(hardwareMap, "W", 0, 260, AngleUnit.DEGREES);

        V4B1.setInverted(true);

        V4B1.turnToAngle(90);
        V4B2.turnToAngle(90);
        Wrist.turnToAngle(defaultWrist);

        Timer timer = new Timer(2);
        timer.start();

        while (!timer.done()) {
        }

        resetPosition();
    }

    public void placing() {
        V4B1.turnToAngle(250);
        V4B2.turnToAngle(250);
    }

    public void manualV4BControl(double angle, Telemetry telemetry) {
        int scaling = 8;

        V4B1.rotateByAngle(-angle  * scaling);
        V4B2.rotateByAngle(-angle  * scaling);

        telemetry.addData("V4B: ", V4B1.getAngle());
    }

    public void manualWristControl(double angle, Telemetry telemetry) {
        int scaling = 5;

        Wrist.rotateByAngle(-angle  * scaling);

        telemetry.addData("Wrist: ", Wrist.getAngle());
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
