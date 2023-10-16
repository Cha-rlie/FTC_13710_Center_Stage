package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase extends SubsystemBase {

    // Declare drivebase motor variables
    MotorEx frontLeft;
    MotorEx frontRight;
    MotorEx rearLeft;
    MotorEx rearRight;

    // Declare the drivebase
    MecanumDrive drivebase;

    public Drivebase(HardwareMap hardwareMap) {
        // Assign corresponding values to the MotorEx objects
        frontLeft = new MotorEx(hardwareMap, "FrontLeft");
        frontRight = new MotorEx(hardwareMap, "FrontRight");
        rearLeft =  new MotorEx(hardwareMap, "RearLeft");
        rearRight = new MotorEx(hardwareMap, "RearRight");

        // Assign a new mecanum drivebase to the driveBase variable
        drivebase = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);

    }

    public void userControlledDrive(GamepadEx driveOp, double gyroAngle) {
        // Drive the robot using gamepad input in a field-centric way
        drivebase.driveFieldCentric(driveOp.getLeftX(), driveOp.getLeftY(), driveOp.getRightX(), gyroAngle);
    }

    // More Functions Could be Added Here (Especially for Auto)

}
