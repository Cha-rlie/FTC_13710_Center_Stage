package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

public class Intake extends SubsystemBase {

    public final Motor intakeSpinner;
    public Boolean intakeCurrentlySpinning = false;
    private GamepadEx toolOp;
    private Timer timer = new Timer(5, TimeUnit.SECONDS);
    public ServoEx IntakeCover;
    public ElapsedTime coverTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap, GamepadEx toolop) {
        // Initialising the Motors
        intakeSpinner = new Motor(hardwareMap, "Spinner");
        intakeSpinner.setRunMode(Motor.RunMode.RawPower);
        intakeSpinner.setInverted(false);

        IntakeCover = new SimpleServo(hardwareMap, "I", 0, 260, AngleUnit.DEGREES);

    }

    public void spin() {
        intakeSpinner.set(-1);
        intakeCurrentlySpinning = true;
    }

    public void Rspin() {
        intakeSpinner.set(1);
        intakeCurrentlySpinning = true;
    }

    public void manualCoverControl(double angle, Telemetry telemetry) {
        int scaling = 8;

        IntakeCover.rotateByAngle(angle  * scaling);

        telemetry.addData("Intake Cover: ", IntakeCover.getAngle());
    }

    public void spinFor5Seconds() {
        if (intakeCurrentlySpinning) {

            if (!timer.isTimerOn()) {
                timer.start();
            }

            intakeSpinner.set(0);

            if (timer.done()) {
                intakeSpinner.set(0);
                intakeCurrentlySpinning = false;
            }
        }
    }

//    @Override
//    public void periodic() {
//        if (intakeCurrentlySpinning) {
//            intakeSpinner.set(0.7);
//        } else {
//            intakeSpinner.set(0);
//        }
//
//    }
}
