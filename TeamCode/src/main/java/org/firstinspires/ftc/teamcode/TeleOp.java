package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Import local FTCLib hardware classes
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

// Import local FTCLib command classes


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
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
    boolean buttonIsReleased;

    Encoder parallelEncoder;
    Encoder perpendicularEncoder;


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
        shooter = new DroneLauncher(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();

        CommandScheduler.getInstance();

        clawClosed = false;
        buttonIsReleased = true;

         parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RearRight"));
         perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RearLeft"));
         perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
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
        //CommandScheduler.getInstance().run();

        telemetry.addData("PAR", parallelEncoder.getCurrentPosition());
        telemetry.addData("PER", perpendicularEncoder.getCurrentPosition());
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

        double totalCurrent = driveBase.frontLeft.getCurrent(CurrentUnit.AMPS) + driveBase.frontRight.getCurrent(CurrentUnit.AMPS) + driveBase.rearLeft.getCurrent(CurrentUnit.AMPS) + driveBase.rearRight.getCurrent(CurrentUnit.AMPS);
        telemetry.addData("Total Current Draw: ", totalCurrent);


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
        }

        // Complex transfer command
        if(commandRun) {
            deposit.Wrist.turnToAngle(245);
            if(deposit.safeTimer.seconds() > 0.5 && deposit.safeTimer.seconds() < 0.8) {
                deposit.V4B.turnToAngle(deposit.rampPosition);
            } else if(deposit.safeTimer.seconds() > 0.8 && deposit.safeTimer.seconds() < 1.2) {
                deposit.Gripper.turnToAngle(deposit.closedPosition);
            } else if(deposit.safeTimer.seconds() > 1.2 && deposit.safeTimer.seconds() < 2) {
                intake.IntakeCover.turnToAngle(114);
                commandRun = false;
            }
        }



        if(driveOp.getButton(GamepadKeys.Button.DPAD_UP )) {
            hang.raise();
        } else if (driveOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            hang.lower();
        } else {
            hang.Hang.motor.setPower(0);
            hang.HangPusher.setPower(0);
        }


        if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            deposit.upSlides();
            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
        } else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            deposit.downSlides();
            deposit.lastSlidePos = deposit.DS1.getCurrentPosition();
        } else {
            deposit.powerOffSlides();
        }

        if(toolOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            deposit.mosaicSpin(1, telemetry);
        } else if(toolOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            deposit.mosaicSpin(-1, telemetry);
        } else if(toolOp.getButton(GamepadKeys.Button.BACK)) {
            deposit.mosaicSpin(0, telemetry);
        }

        // Safety Features
//        if(deposit.getV4BPos() < 100 && deposit.getV4BPos() > 90) {
//            deposit.Spin.turnToAngle(deposit.flatSpin);
//        } else if(deposit.getV4BPos() > 100) {
//            deposit.Spin.turnToAngle(deposit.transferSpin);
//        }

        if (deposit.V4B.getAngle() < 180) {
            deposit.Spin.turnToAngle(deposit.transferSpin);
        }



        deposit.manualV4BControl(toolOp.getLeftY(), telemetry);
        deposit.manualWristControl(toolOp.getRightY(), telemetry);

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