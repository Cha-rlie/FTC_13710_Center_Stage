package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


// THIS IS FOR FUTURE USE, BUT IT IS NOT NEEDED RIGHT NOW
// This still needs a fair amount of work, but it's a good start I think :)

public class Transfer extends CommandBase {

    private final Deposit depositSubSystem;

    private final GamepadEx gamePad1;

    public Transfer(HardwareMap hardwareMap, GamepadEx gamepad1) {
        depositSubSystem = new Deposit(hardwareMap);
        gamePad1 = gamepad1;
    }

    @Override
    public void initialize() {
        // I'm not sure this is needed, so I left it empty, but you can change that :)
        depositSubSystem.rotateGripperAngleToPickUp();
        depositSubSystem.pickUp();
        depositSubSystem.grip();
        depositSubSystem.rotateGripperAngleToDeposit();
        depositSubSystem.placing();
        depositSubSystem.letGo();
    }

    @Override
    public boolean isFinished() {
        return true; // This will mean the initialise code will run just once which is what we want
    }

}
