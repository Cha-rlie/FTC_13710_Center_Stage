package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Import local FTCLib hardware classes
import org.firstinspires.ftc.teamcode.commands.Reset;
import org.firstinspires.ftc.teamcode.commands.Serve;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;

// Import local FTCLib command classes
import org.firstinspires.ftc.teamcode.commands.Omelette;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    Drivebase driveBase;
    IMU imu;
    GamepadEx driveOp;
    GamepadEx toolOp;

    public boolean willResetIMU = true;
    ElapsedTime timer = new ElapsedTime();
    boolean commandRun;

    // Declaring Commands
    private Deposit deposit;
    private Intake intake;
    private Hang hang;

    public void init() {
        driveBase = new Drivebase(hardwareMap);
        // Initialise the imuGyro with the correct orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
        commandRun = false;
        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        hang = new Hang(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();

        CommandScheduler.getInstance();

    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {

    }

    public void start () {

    }

    public void loop() {
        // This must be called from the function that loops in the opmode for everything to run
        CommandScheduler.getInstance().run();

        // Run the drivebase with the driveOp gamepad
        driveBase.userControlledDrive(driveOp, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // Reset the imu if the driver deems it necessary
        if (gamepad1.guide) {
            imu.resetYaw();
            telemetry.addLine("Imu Reset");
        }

        // Update the variables
        if (driveOp.isDown(GamepadKeys.Button.A)) {
            intake.spin();
        } else if(driveOp.isDown(GamepadKeys.Button.B)) {
            intake.Rspin();
        } else {
            intake.spinFor5Seconds();
        }

        // Puts back to the transfer position
//        Button transferButton = new GamepadButton(toolOp, GamepadKeys.Button.B);
//        transferButton.whenPressed(new Transfer(deposit));

        // Flips the pan over
//        Button flipButtom = new GamepadButton(toolOp, GamepadKeys.Button.A);
//        flipButtom.whenPressed(new Omelette(deposit));


//        if(toolOp.getButton(GamepadKeys.Button.A)) {
//            deposit.Gripper.turnToAngle(50);
//        } else {
//            deposit.Gripper.turnToAngle(140);
//        }

        if(toolOp.getButton(GamepadKeys.Button.X)) {
            deposit.home();
            commandRun = true;
            deposit.safeTimer.reset();
        }

        if(commandRun) {
            if(deposit.safeTimer.seconds() > 1 && deposit.safeTimer.seconds() < 2) {
                deposit.Gripper.turnToAngle(50);
            } else if(deposit.safeTimer.seconds() > 2 && deposit.safeTimer.seconds() < 3) {
                deposit.safe();
            } else if(deposit.safeTimer.seconds() > 3 && deposit.safeTimer.seconds() < 4) {
                deposit.place();
                commandRun = false;
                }
        }


        // Reset
        if(toolOp.getButton(GamepadKeys.Button.Y)) {
            deposit.home();
        }


        if(driveOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            hang.raise();
        } else if (driveOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            hang.lower();
        }


        if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            deposit.upSlides();
        } else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            deposit.downSlides();
        } else {
            deposit.powerOffSlides();
        }


        deposit.manualV4BControl(toolOp.getLeftY(), telemetry);
        deposit.manualWristControl(toolOp.getRightY(), telemetry);


        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
//        telemetry.addData("Hang Pos", hang.Hang.getCurrentPosition());
        telemetry.addData("Slide Pos 1", deposit.DS1.motor.getCurrentPosition());
        telemetry.addData("Slide Pos 2", deposit.DS2.motor.getCurrentPosition());
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}