package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    public final Motor intakeSpinner;

    private Boolean intakeCurrentlySpinning = false;

    private GamepadEx gamePad1;

    public Intake(HardwareMap hardwareMap, GamepadEx gamepad1) {
        // Initialising the Motors
        intakeSpinner = new Motor(hardwareMap, "Spinner");
        intakeSpinner.setRunMode(Motor.RunMode.RawPower);

        // Initialising the Gamepad Button Readers
        gamePad1 = gamepad1;

    }

    @Override
    public void periodic() {

        // This must be called from the periodic() function of each subsystem for it to be run
        CommandScheduler.getInstance().run();

//        if (gamePad1.isDown(GamepadKeys.Button.A) && !gamePad1.wasJustPressed(GamepadKeys.Button.A)) {
////
//            intakeCurrentlySpinning = !intakeCurrentlySpinning;
//
////            if (intakeCurrentlySpinning) {
////                intakeCurrentlySpinning = false;
////            } else {
////                intakeCurrentlySpinning = true;
////            }
//        }
//
//        if (intakeCurrentlySpinning) {
//            intakeSpinner.set(1);
//        } else {
//            intakeSpinner.set(0);
//        }

        intakeSpinner.set(1);

    }
}
