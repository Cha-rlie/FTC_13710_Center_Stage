package org.firstinspires.ftc.teamcode.hardware.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

class IntakeSubSystem extends SubsystemBase {

    public final Motor intakeSpinner;

    private Boolean intakeCurrentlySpinning = false;

    public IntakeSubSystem(HardwareMap hardwareMap) {
        // Initialising the Motors
        intakeSpinner = new Motor(hardwareMap, "IntakeSpinner1");
        intakeSpinner.setRunMode(Motor.RunMode.RawPower);

        // Initialising the Gamepad Button Readers


    }

    public void spin(GamepadEx gamePad1) {

        if (gamePad1.isDown(GamepadKeys.Button.A) && !gamePad1.wasJustPressed(GamepadKeys.Button.A)) {
//
          intakeCurrentlySpinning = !intakeCurrentlySpinning;

//            if (intakeCurrentlySpinning) {
//                intakeCurrentlySpinning = false;
//            } else {
//                intakeCurrentlySpinning = true;
//            }
        }

        if (intakeCurrentlySpinning) {
            intakeSpinner.set(1);
        } else {
            intakeSpinner.set(0);
        }

        /* Timer timer = new Timer(2, TimeUnit.SECONDS);

        intakeMotors.set(1);
        timer.start();

        while (timer.elapsedTime() < 10) {

        }

        intakeMotors.set(0); */

    }

//    @Override
//    public void periodic() {
//
//    }

}

public class Intake extends CommandBase {

    private final IntakeSubSystem intakeSubSystem;

    private final GamepadEx gamePad1;

    public Intake(HardwareMap hardwareMap, GamepadEx gamepad1) {
        intakeSubSystem = new IntakeSubSystem(hardwareMap);
        gamePad1 = gamepad1;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubSystem.spin(gamePad1);
    }

}
