package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.FlattenCommand;
import org.firstinspires.ftc.teamcode.commands.HomeCommand;

@Config
public class LiftSubsystem extends SubsystemBase {
    protected DepositSubsystem deposit;
    protected LiftSubsystem lift;
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    protected SlideModel slideModel;
    Telemetry telemetry;
    Gamepad gamepad2;

    public static int top;
    public double liftOffset = 0;


    public LiftSubsystem(HardwareMap hMap, Telemetry telemetry, Gamepad gamepad2, DepositSubsystem deposit, LiftSubsystem lift){
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
        this.deposit = deposit;
        this.lift = lift;

        slideModel = new SlideModel();

        leftMotor = new MotorEx(hMap,"DS1", Motor.GoBILDA.RPM_1150);
        rightMotor = new MotorEx(hMap, "DS2", Motor.GoBILDA.RPM_1150);
        leftMotor.setInverted(true);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top = slideModel.MAXENCODER;
    }

    @Override
    public void periodic() {
        double power = 1;
        if(gamepad2.touchpad_finger_1 && gamepad2.touchpad_finger_2) {
            new HomeCommand(deposit, lift).schedule();
        } else if(gamepad2.touchpad_finger_1) {
            int[] realWorldLoc = slideModel.convertInputToLocation(gamepad2.touchpad_finger_1_x, gamepad2.touchpad_finger_1_y, telemetry);

            double[] liftValues = slideModel.inverseKinematics(realWorldLoc[0], realWorldLoc[1]);

            if(liftValues != null) {
                leftMotor.motor.setTargetPosition((int) liftValues[0]);
                rightMotor.motor.setTargetPosition((int) liftValues[0]);

                leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftMotor.motor.setPower(power);
                rightMotor.motor.setPower(power);

                double armPosition = Math.toDegrees(liftValues[1]) + 90;
                telemetry.addData("Desired Arm Pos: ", armPosition);
                deposit.V4B.turnToAngle(armPosition);

                double difference = deposit.V4B.getAngle() - 60;
                deposit.Wrist.turnToAngle(180-difference);

                new FlattenCommand(deposit).schedule();
            }
        }
    }

    public void run(double joystick) {
        leftMotor.motor.setTargetPosition(target(joystick));
        rightMotor.motor.setTargetPosition(target(joystick));

        telemetry.addData("Target: ", target(joystick));
        telemetry.addData("Power: ", Math.abs(joystick));

        leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.motor.setPower(Math.abs(joystick));
        rightMotor.motor.setPower(Math.abs(joystick));

    }

    public int target(double joystick) { if(joystick > 0) return top; else return 0; };

    public double getPosition(){
        return leftMotor.getCurrentPosition();
    }
}