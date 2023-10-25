package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import java.util.concurrent.TimeUnit;

// Import Local Custom Classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;

public class Omelette extends CommandBase {

    private final Deposit depositSubSystem;
    private Timing.Timer timer;

    public Omelette(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {
        this.timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
        this.timer.start();
        depositSubSystem.Wrist.turnToAngle(70);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (this.timer.done()) {
            depositSubSystem.Wrist.turnToAngle(depositSubSystem.defaultWrist);
            return true;
        } else {
            return false;
        }
    }
}
