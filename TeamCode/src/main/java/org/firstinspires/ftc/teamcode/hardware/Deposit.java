package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Deposit extends SubsystemBase {

    // Servos
    public ServoEx Wrist; // Wrist
    public ServoEx Gripper;
    public ServoEx V4B;
    public AnalogInput V4B_Analog;
    public ServoEx Spin;


    // Slide Motors
    public MotorEx DS1; // Deposit Slide 1
    public MotorEx DS2; // Deposit Slide 2

    public MotorGroup DS; // Deposit Slides
    // Slide positions
    public int min = 0;
    public int max = 1300;
    public double power = 1;

    public int lastSlidePos = 0;
    public double defaultWrist = 205;
    public double rampPosition = 128.4;
    public int closedPosition = 140;
    public int openPosition = 30;
    public int transferSpin = 27;
    public int flatSpin = 120;
    public ElapsedTime safeTimer = new ElapsedTime();

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);

        DS1 = new MotorEx(hardwareMap, "DS1", Motor.GoBILDA.RPM_1150);
        DS2 = new MotorEx(hardwareMap, "DS2", Motor.GoBILDA.RPM_1150);
        DS1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DS2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DS1.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        int MIN_ANGLE = 0;
        int MAX_ANGLE = 355;

        V4B = new SimpleServo(hardwareMap, "V4B", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        V4B_Analog = hardwareMap.get(AnalogInput.class, "VA");


        Wrist = new SimpleServo(hardwareMap, "W", 0, 260, AngleUnit.DEGREES);
        Gripper = new SimpleServo(hardwareMap, "G", 0, 180, AngleUnit.DEGREES);
        Spin = new SimpleServo(hardwareMap, "Spin", 0, 180, AngleUnit.DEGREES);

        V4B.turnToAngle(150);

        Wrist.turnToAngle(defaultWrist);
        Gripper.turnToAngle(openPosition);
        Spin.turnToAngle(transferSpin);

        resetPosition();
    }

    public double getV4BPos() {
        return(V4B_Analog.getVoltage() / 3.3 * 360);
    }


    public void manualV4BControl(double angle, Telemetry telemetry) {
        int scaling = 8;

        V4B.rotateByAngle(angle  * scaling);

        telemetry.addData("V4B: ", V4B.getAngle());
    }

    public void manualWristControl(double angle, Telemetry telemetry) {
        int scaling = 5;

        Wrist.rotateByAngle(-angle  * scaling);

        telemetry.addData("Wrist: ", Wrist.getAngle());
    }

    public void manualSpinControl(double angle, Telemetry telemetry) {
        int scaling = 5;

        Spin.rotateByAngle(-angle  * scaling);

        telemetry.addData("Spin: ", Spin.getAngle());
    }

    public void mosaicSpin(double direction, Telemetry telemetry) {
        if(direction == 1) {
            Spin.turnToAngle(flatSpin - 60);
        } else if (direction == -1){
            Spin.turnToAngle(flatSpin + 60);
        } else {
            Spin.turnToAngle(flatSpin);
        }
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

    public void home() {
        V4B.turnToAngle(145);
        Wrist.turnToAngle(260);
        Spin.turnToAngle(transferSpin);
    }

    public void place() {
        V4B.turnToAngle(324);
        Wrist.turnToAngle(168);
    }

    public void upSlides() {
        DS1.motor.setTargetPosition(max);
        DS2.motor.setTargetPosition(max);

        DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DS1.motor.setPower(power);
        DS2.motor.setPower(power);
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
//        DS1.motor.setPower(0);
//        DS2.motor.setPower(0);

        DS1.motor.setTargetPosition(lastSlidePos);
        DS2.motor.setTargetPosition(lastSlidePos);

        DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DS1.motor.setPower(power);
        DS2.motor.setPower(power);
    }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        if((currentPos < wantedPos + range) && currentPos > wantedPos - range) {
            return true;
        } else {
            return false;
        }
    }
}
