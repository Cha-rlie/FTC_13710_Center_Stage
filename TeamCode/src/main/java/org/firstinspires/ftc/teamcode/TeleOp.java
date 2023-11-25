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
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
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
    boolean IntakeOneTime = false;
    boolean raised;

    // Declaring Commands
    private Deposit deposit;
    private Intake intake;
    private Hang hang;
    private DroneLauncher shooter;

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
        raised = false;

        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        hang = new Hang(hardwareMap);
        shooter = new DroneLauncher(hardwareMap);
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
        //CommandScheduler.getInstance().run();

        // Run the drivebase with the driveOp gamepad
        driveBase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Reset the imu if the driver deems it necessary
        if (gamepad1.guide) {
            imu.resetYaw();
            telemetry.addLine("Imu Reset");
        }


        // Update the variables
        if (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            intake.IntakeCover.turnToAngle(200);
            deposit.V4B1.turnToAngle(90);
            deposit.V4B2.turnToAngle(90);
            if(!IntakeOneTime) {
                intake.coverTimer.reset();
                IntakeOneTime = true;
            }

            if(IntakeOneTime && intake.coverTimer.milliseconds() > 200) {
                intake.spin();
            }
        } else if(toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            intake.Rspin();
        } else {
            intake.intakeSpinner.motor.setPower(0);
            intake.IntakeCover.turnToAngle(130);
            IntakeOneTime = false;
        }


        if(gamepad2.guide) {
            shooter.shoot();
        }

        // Release
        if(toolOp.getButton(GamepadKeys.Button.A)) {
            deposit.Gripper.turnToAngle(deposit.openPosition);
        }

        // X goes to scoring position
        if(toolOp.getButton(GamepadKeys.Button.X)) {
            deposit.place();
        }

        // Y goes to transfer position
        if(toolOp.getButton(GamepadKeys.Button.Y)) {
            deposit.home();
        }

        // B completes transfer
        if(toolOp.getButton(GamepadKeys.Button.B)) {
            commandRun = true;
            deposit.safeTimer.reset();
        }

        // Complex transfer command
        if(commandRun) {
            deposit.Wrist.turnToAngle(deposit.defaultWrist);
            if(deposit.safeTimer.seconds() > 0 && deposit.safeTimer.seconds() < 0.5) {
                deposit.V4B1.turnToAngle(deposit.rampPosition);
                deposit.V4B2.turnToAngle(deposit.rampPosition);
            } else if(deposit.safeTimer.seconds() > 0.5 && deposit.safeTimer.seconds() < 1) {
                deposit.Gripper.turnToAngle(deposit.closedPosition);
            } else if(deposit.safeTimer.seconds() > 1 && deposit.safeTimer.seconds() < 1.5) {
                deposit.safe();
            } else if(deposit.safeTimer.seconds() > 1.5) {
                deposit.visible();
                commandRun = false;
            }
        }

//        if(driveOp.getButton(GamepadKeys.Button.DPAD_UP ) || raised) {
//            hang.raise();
//            raised = true;
//        } else if (driveOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            hang.lower();
//            raised = false;
//        }


        if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            deposit.upSlides();
        } else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            deposit.downSlides();
        } else {
            deposit.powerOffSlides();
        }

        telemetry.addData("Speed Modifier: ", driveBase.speedModifier);


        deposit.manualV4BControl(-toolOp.getLeftY(), telemetry);
        deposit.manualWristControl(toolOp.getRightY(), telemetry);


        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
//        telemetry.addData("Hang Pos", hang.Hang.getCurrentPosition());
        telemetry.addData("Slide Pos 1", deposit.DS1.motor.getCurrentPosition());
        telemetry.addData("Slide Pos 2", deposit.DS2.motor.getCurrentPosition());
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("FrontLeft", driveBase.frontLeft.getCurrentPosition());
        telemetry.addData("FrontRight", driveBase.frontRight.getCurrentPosition());
        telemetry.addData("RearLeft", driveBase.rearLeft.getCurrentPosition());
        telemetry.addData("RearRight", driveBase.rearRight.getCurrentPosition());
        telemetry.update();
    }
}