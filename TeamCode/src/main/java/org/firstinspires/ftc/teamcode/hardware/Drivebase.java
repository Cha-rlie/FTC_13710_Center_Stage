package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase extends SubsystemBase {

    // Declare drivebase motor variables
    MotorEx frontLeft;
    MotorEx frontRight;
    MotorEx rearLeft;
    MotorEx rearRight;

    // Declare the drivebase
    MecanumDrive drivebase;

    // Declare variables
    double speedModifier;

    public Drivebase(HardwareMap hardwareMap) {
        // Assign corresponding values to the MotorEx objects with the correct RPMs since they are GoBildas
        frontLeft = new MotorEx(hardwareMap, "FrontLeft", Motor.GoBILDA.RPM_223);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight = new MotorEx(hardwareMap, "FrontRight", Motor.GoBILDA.RPM_223);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearLeft =  new MotorEx(hardwareMap, "RearLeft", Motor.GoBILDA.RPM_223);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearRight = new MotorEx(hardwareMap, "RearRight", Motor.GoBILDA.RPM_223);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        rearRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Assign a new mecanum drivebase to the driveBase variable
        drivebase = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);

    }

    public void userControlledDrive(GamepadEx driveOp, double heading) {
        if (driveOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get()) {
            speedModifier = 0.35;
        } else if (driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get()) {
            speedModifier = 1;
        } else {
            speedModifier = 0.7;
        }
        // Drive the robot using gamepad input in a field-centric way
        drivebase.driveFieldCentric(-driveOp.getLeftX()*speedModifier,
                -driveOp.getLeftY()*speedModifier,
                -driveOp.getRightX()*speedModifier,
                heading);
    }

    // More Functions Could be Added Here (Especially for Auto)

}
