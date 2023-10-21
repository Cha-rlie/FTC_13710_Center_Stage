package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    DriveBase driveBase;
    IMU imuGyro;
    GamepadEx driveOp;
    GamepadEx toolOp;

    public boolean willResetIMU = true;

    // Declaring Commands
    private Deposit deposit;
    private Intake intake;
    //private Hang hang;

    public void init() {
        driveBase = new DriveBase(hardwareMap);
        // Initialise the imuGyro with the correct orientation
        /*imuGyro.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        ));*/
        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        //hang = new Hang(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();

        CommandScheduler.getInstance();

    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (driveOp.getButton(GamepadKeys.Button.Y)) {
            willResetIMU = false;
        }
    }

    public void start () {
        //if (willResetIMU) imuGyro.resetYaw();
    }

    public void loop() {

        // This must be called from the function that loops in the opmode for everything to run
        CommandScheduler.getInstance().run();

        // Run the drivebase with the driveOp gamepad
        driveBase.userControlledDrive(driveOp);

        // Update the variables
        if (driveOp.isDown(GamepadKeys.Button.A)) {
            //intake.spin();
        } else if (!driveOp.isDown(GamepadKeys.Button.A)) {
            //intake.spinFor5Seconds();
        }

       Button transferButton = new GamepadButton(
                toolOp, GamepadKeys.Button.B
        );
       transferButton.whenPressed(new Transfer(deposit));

        if (driveOp.getButton(GamepadKeys.Button.X)) {
            deposit.placing();
        } else if (driveOp.getButton(GamepadKeys.Button.A)) {
            deposit.grip();
        }


//        if (toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
//            deposit.upSlides();
//        } else if (toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            deposit.downSlides();
//        } else {
//            deposit.powerOffSlides();
//        }

//        if(hang.hung) {
//            hang.Hang.motor.setPower(0.2);
//        }

        deposit.manualV4BControl(toolOp.getRightY(), telemetry);
        deposit.manualWristControl(toolOp.getLeftY(), telemetry);
//
//        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
//        telemetry.addData("Hang Pos", hang.Hang.getCurrentPosition());
//        telemetry.addData("Slide Pos 1: ", deposit.DS1.motor.getCurrentPosition());
//        telemetry.addData("Slide Pos 2: ", deposit.DS2.motor.getCurrentPosition());
        telemetry.update();
    }
}