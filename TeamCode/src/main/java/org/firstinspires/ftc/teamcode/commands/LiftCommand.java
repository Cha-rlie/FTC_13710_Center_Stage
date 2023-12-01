package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.commands.Lift;

public class LiftCommand extends CommandBase {
    private final Lift lift;
    private final double targetPosition;

    public LiftCommand(Lift lift, double targetPosition) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        lift.target = targetPosition;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.target - lift.getPosition()) < 10;
    }
}