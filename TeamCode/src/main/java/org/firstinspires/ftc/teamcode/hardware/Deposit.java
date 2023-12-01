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


    public MotorGroup DS; // Deposit Slides
    // Slide positions
    public int lastSlidePos = 0;
    public double defaultWrist = 205;
    public double rampPosition = 131.53;
    public int closedPosition = 150;
    public int openPosition = 30;
    public int transferSpin = 27;
    public int flatSpin = 120;
    public ElapsedTime safeTimer = new ElapsedTime();

    public Deposit(HardwareMap hardwareMap) {
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);

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

        ElapsedTime wait = new ElapsedTime();
        wait.reset();

        while(wait.seconds() < 2) {
        }

        coverSafeMove();
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



    public void home() {
        V4B.turnToAngle(160);
        Wrist.turnToAngle(260);
        Spin.turnToAngle(transferSpin);
    }

    public void coverSafeMove() {
        // Moves the deposit if its going to intercept the cover movement
        if(V4B.getAngle() < 160) { V4B.turnToAngle(160); }
        if(Wrist.getAngle() < 175) { Wrist.turnToAngle(175); }
    }

    public void place() {
        V4B.turnToAngle(324);
        Wrist.turnToAngle(165);
    }

    public void grab() { Gripper.turnToAngle(closedPosition); }
    public void release() { Gripper.turnToAngle(openPosition); }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        if((currentPos < wantedPos + range) && currentPos > wantedPos - range) {
            return true;
        } else {
            return false;
        }
    }
}
