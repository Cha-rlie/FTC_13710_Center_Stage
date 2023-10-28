package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import Local Custom Classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;


// THIS IS FOR FUTURE USE, BUT IT IS NOT NEEDED RIGHT NOW
// This still needs a fair amount of work, but it's a good start I think :)

public class Reset extends CommandBase {

    private final Deposit depositSubSystem;
    boolean operationFinished;
    private ElapsedTime timer1 = new ElapsedTime();
    boolean oneTime;
    private double power = 0.5;

    public Reset(Deposit deposit) {
        depositSubSystem = deposit;
    }

    @Override
    public void initialize() {
        operationFinished = false;
        oneTime = false;
    }

    @Override
    public void execute() {
        int currentLocation = (depositSubSystem.DS1.motor.getCurrentPosition()
                + depositSubSystem.DS2.motor.getCurrentPosition())/2;
        int uncertainty = 10;

        if (!depositSubSystem.withinUncertainty(currentLocation, depositSubSystem.min, uncertainty)) {
            depositSubSystem.DS1.motor.setTargetPosition(depositSubSystem.min);
            depositSubSystem.DS2.motor.setTargetPosition(depositSubSystem.min);

            depositSubSystem.DS1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            depositSubSystem.DS2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            depositSubSystem.DS1.motor.setPower(power);
            depositSubSystem.DS2.motor.setPower(power);
        } else {
            if(!oneTime) {
                depositSubSystem.Wrist.turnToAngle(120);
                depositSubSystem.V4B1.turnToAngle(depositSubSystem.rampPosition);
                depositSubSystem.V4B2.turnToAngle(depositSubSystem.rampPosition);

                operationFinished = true;

                timer1.reset();
                oneTime = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(operationFinished) {
            if(timer1.seconds() > 1) {
                depositSubSystem.Wrist.turnToAngle(depositSubSystem.defaultWrist);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}
