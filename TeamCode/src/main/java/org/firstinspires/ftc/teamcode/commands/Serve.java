package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Import Local Custom Classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;


public class Serve extends CommandBase {

    private final Deposit depositSubSystem;
    private int targetPos = 0;
    boolean operationFinished;
    double increment;

    public Serve(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {
        operationFinished = false;
        increment = depositSubSystem.V4B1.getAngle();
    }

    @Override
    public void execute() {
        int currentLocation = (depositSubSystem.DS1.motor.getCurrentPosition()
                + depositSubSystem.DS2.motor.getCurrentPosition())/2;
        int safePos = 200;
        int uncertainty = 10;

        // Determine whether the outtake is past the safe rotation position
        if(currentLocation >= safePos-(uncertainty*2)) {
            depositSubSystem.Wrist.turnToAngle(185);

            if(increment < 180) {
                depositSubSystem.V4B1.rotateByAngle(1);
                depositSubSystem.V4B2.rotateByAngle(1);
                increment += 1;
            } else {
                operationFinished = true;
            }

        } else {
            depositSubSystem.DS1.motor.setTargetPosition(safePos);
            depositSubSystem.DS2.motor.setTargetPosition(safePos);

            depositSubSystem.DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            depositSubSystem.DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            depositSubSystem.DS1.motor.setPower(1);
            depositSubSystem.DS2.motor.setPower(1);
        }
    }

    @Override
    public boolean isFinished() {
        if(operationFinished) {
            return true;
        } else {
            return false;
        }
    }
}
