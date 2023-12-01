package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Import local FTCLib hardware classes
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;

// Import local FTCLib command classes


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends CommandOpMode {
    Drivebase driveBase;
    IMU imu;
    GamepadEx driveOp;
    GamepadEx toolOp;

    public boolean willResetIMU = true;
    ElapsedTime timer = new ElapsedTime();
    boolean commandRun;
    boolean IntakeOneTime = false;

    // Declaring Commands
    private Deposit deposit;
    private Intake intake;
    private Hang hang;
    private DroneLauncher shooter;

    boolean clawClosed;
    boolean coverDown;
    boolean buttonIsReleased;

    @Override
    public void initialize() {
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
        new GamepadButton(toolOp, GamepadKeys.Button.START).toggleWhenPressed(() -> intake.openCover(), () -> intake.closeCover());


        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        hang = new Hang(hardwareMap);
        shooter = new DroneLauncher(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();

        CommandScheduler.getInstance();

        clawClosed = false;
        coverDown = true;
        buttonIsReleased = true;
    }
    
    @Override
    public void run() {
        // This must be called from the function that loops in the opmode for everything to run
        CommandScheduler.getInstance().run();

        // Run the drivebase with the driveOp gamepad
        driveBase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Reset the imu if the driver deems it necessary
        if (gamepad1.guide) {
            imu.resetYaw();
            telemetry.addLine("Imu Reset");
        }


        // Update the variables
        if (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            intake.spin();
        } else if(toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            intake.Rspin();
        } else {
            intake.intakeSpinner.motor.setPower(0);
            //intake.IntakeCover.turnToAngle(120);
        }


        if(gamepad2.guide) {
            shooter.shoot();
        }

        if(toolOp.getButton(GamepadKeys.Button.A)) {
            if (buttonIsReleased) {
                buttonIsReleased = false;
                if(clawClosed == false) {
                    clawClosed = true;
                    deposit.Gripper.turnToAngle(deposit.closedPosition);
                } else if(clawClosed == true) {
                    clawClosed = false;
                    deposit.Gripper.turnToAngle(deposit.openPosition);
                }
            }
        } else {
            buttonIsReleased = true;
        }

        // X goes to scoring position
        if(toolOp.getButton(GamepadKeys.Button.X)) {
            deposit.place();
        }

        // Y goes to home position
        if(toolOp.getButton(GamepadKeys.Button.Y)) {
            deposit.home();
        }

        // B completes transfer
        if(toolOp.getButton(GamepadKeys.Button.B)) {
            commandRun = true;
            deposit.safeTimer.reset();
            deposit.Gripper.turnToAngle(deposit.openPosition);
            deposit.V4B.turnToAngle(160);
        }

        // Complex transfer command
        if(commandRun) {
            deposit.Wrist.turnToAngle(241.9);
            if(deposit.safeTimer.seconds() > 0.5 && deposit.safeTimer.seconds() < 0.8) {
                deposit.V4B.turnToAngle(deposit.rampPosition);
            } else if(deposit.safeTimer.seconds() > 0.8 && deposit.safeTimer.seconds() < 1.2) {
                deposit.Gripper.turnToAngle(deposit.closedPosition);
            } else if(deposit.safeTimer.seconds() > 1.2 && deposit.safeTimer.seconds() < 2) {
                commandRun = false;
                deposit.V4B.turnToAngle(170);
                intake.openCover();
                coverDown = true;
            }
        }

//        if(toolOp.getButton(GamepadKeys.Button.START)) {
//            if (buttonIsReleased) {
//                buttonIsReleased = false;
//                if(coverDown == false) {
//                    coverDown = true;
//                    deposit.coverSafeMove();
//                    intake.closeCover();
//                } else if(coverDown == true) {
//                    coverDown = false;
//                    deposit.coverSafeMove();
//                    intake.openCover();
//                }
//            }
//        } else {
//            buttonIsReleased = true;
//        }

//        if(driveOp.getButton(GamepadKeys.Button.DPAD_UP )) {
//            hang.raise();
//        } else if (driveOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            hang.lower();
//        } else {
//            hang.Hang.motor.setPower(0);
//            hang.HangPusher.setPower(0);
//        }


//        if(driveOp.getButton(GamepadKeys.Button.DPAD_UP)) {
//            deposit.upSlides();
//            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
//        } else if(driveOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            deposit.downSlides();
//            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
//        } else {
//            deposit.powerOffSlides();
//        }

        if(toolOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            deposit.mosaicSpin(1, telemetry);
        } else if(toolOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            deposit.mosaicSpin(-1, telemetry);
        } else if(toolOp.getButton(GamepadKeys.Button.BACK)) {
            deposit.mosaicSpin(0, telemetry);
        }


        if (deposit.V4B.getAngle() < 180) {
            deposit.Spin.turnToAngle(deposit.transferSpin);
        }

        if(toolOp.getLeftY() > 0.1) {
            deposit.upSlides(toolOp.getLeftY());
            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
        } else if(toolOp.getLeftY() < -0.1) {
            deposit.downSlides(-toolOp.getLeftY());
            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
        } else {
            deposit.powerOffSlides();
        }



        deposit.manualV4BControl(-toolOp.getRightY(), telemetry);

        //deposit.manualWristControl(toolOp.getRightY(), telemetry);
        if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            deposit.manualWristControl(1, telemetry);
        } else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            deposit.manualWristControl(-1, telemetry);
        }

        telemetry.addData("Analog V4B Pos", deposit.getV4BPos());

        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
        telemetry.addData("Intake Cover Pos", intake.IntakeCover.getAngle());
//        telemetry.addData("Hang Pos", hang.Hang.getCurrentPosition());
        telemetry.addData("Slide Pos 1", deposit.DS1.motor.getCurrentPosition());
        telemetry.addData("Slide Pos 2", deposit.DS2.motor.getCurrentPosition());
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }
}