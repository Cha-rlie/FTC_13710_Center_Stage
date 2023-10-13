package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


// THIS IS FOR FUTURE USE, BUT IT IS NOT NEEDED RIGHT NOW
// This still needs a fair amount of work, but it's a good start I think :)

public class Transfer extends CommandBase {

    private final Deposit depositSubSystem;
    private int targetPos = 0;

    public Transfer(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        depositSubSystem.DS1.motor.setTargetPosition(targetPos);
        depositSubSystem.DS2.motor.setTargetPosition(targetPos);

        depositSubSystem.DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositSubSystem.DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        depositSubSystem.DS1.motor.setPower(1);
        depositSubSystem.DS2.motor.setPower(1);

        depositSubSystem.V4B1.turnToAngle(6.4);
        depositSubSystem.V4B2.turnToAngle(6.4);
        depositSubSystem.Gripper.turnToAngle(120);
        depositSubSystem.wristPos = 120;

    }

    @Override
    public boolean isFinished() {

        return true; // This will mean the initialise code will run just once which is what we want
    }
}
