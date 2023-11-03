package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase extends SubsystemBase {

    // Declare drivebase motor variables
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    // Declare the drivebase
    MecanumDrive drivebase;

    // Declare variables
    public double speedModifier;

    public Drivebase(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void userControlledDrive(Gamepad gamepad1, double botHeading) {
        double speedModifier = 0.9; //Used to be 0.7

        if (gamepad1.left_bumper) {
            speedModifier = 0.3;
        } else if (gamepad1.right_bumper) {
            speedModifier = 1;
        }

        double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x;
        double rx = gamepad1.left_stick_x;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower * speedModifier);
        rearLeft.setPower(backLeftPower * speedModifier);
        frontRight.setPower(frontRightPower * speedModifier);
        rearRight.setPower(backRightPower * speedModifier);
    }
}
