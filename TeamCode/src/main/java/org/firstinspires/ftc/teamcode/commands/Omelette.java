package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

// Import Local Custom Classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;

public class Omelette extends CommandBase {

    private final Deposit depositSubSystem;
    ElapsedTime timer = new ElapsedTime();

    public Omelette(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {
        timer.reset();
        depositSubSystem.Wrist.turnToAngle(100);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (timer.milliseconds() > 500) {
            depositSubSystem.Wrist.turnToAngle(depositSubSystem.defaultWrist);
            return true;
        } else {
            return false;
        }
    }
}
