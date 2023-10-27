package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Import Local Custom Classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;


// THIS IS FOR FUTURE USE, BUT IT IS NOT NEEDED RIGHT NOW
// This still needs a fair amount of work, but it's a good start I think :)

public class Reset extends CommandBase {

    private final Deposit depositSubSystem;

    public Reset(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        int currentLocation = (depositSubSystem.DS1.motor.getCurrentPosition()
                + depositSubSystem.DS2.motor.getCurrentPosition())/2;
        int uncertainty = 10;

        while (!depositSubSystem.withinUncertainty(currentLocation, depositSubSystem.min, uncertainty)) {
            depositSubSystem.DS1.motor.setTargetPosition(depositSubSystem.min);
            depositSubSystem.DS2.motor.setTargetPosition(depositSubSystem.min);

            depositSubSystem.DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            depositSubSystem.DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            depositSubSystem.DS1.motor.setPower(1);
            depositSubSystem.DS2.motor.setPower(1);
        }
    }

    @Override
    public boolean isFinished() {
        return true; // This will mean the initialise code will run just once which is what we want
    }
}
