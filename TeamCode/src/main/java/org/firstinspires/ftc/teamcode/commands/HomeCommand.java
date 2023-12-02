package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class HomeCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private ElapsedTime timer;

    public HomeCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        deposit.V4B.turnToAngle(270);
        deposit.Wrist.turnToAngle(180);
        deposit.Spin.turnToAngle(deposit.transferSpin);
    }

    @Override
    public void execute() {
        lift.leftMotor.motor.setTargetPosition(0);
        lift.rightMotor.motor.setTargetPosition(0);

        lift.leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.leftMotor.motor.setPower(1);
        lift.rightMotor.motor.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if(lift.leftMotor.getCurrentPosition() < 10) { return true; }
        else return false;
    }
}