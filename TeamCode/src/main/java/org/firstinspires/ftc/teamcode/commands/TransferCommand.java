package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class TransferCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private ElapsedTime timer;
    int delay;

    public TransferCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        deposit.Wrist.turnToAngle(251.5);
        deposit.Spin.turnToAngle(deposit.transferSpin);
        deposit.Gripper.turnToAngle(deposit.transferGrip);
    }

    @Override
    public void execute() {
        delay = 500;

        if(timer.milliseconds() > delay && timer.milliseconds() < delay*2) {
            deposit.V4B.turnToAngle(272);
        } else if (timer.milliseconds() > delay*2 && timer.milliseconds() < delay*3) {
            deposit.grab();
        } else if(timer.milliseconds() > delay*3) {
            intake.openCover();
        }
    }

    @Override
    public void end(boolean interrupted) {
        new HomeCommand(deposit, lift).schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > delay*4;
    }
}