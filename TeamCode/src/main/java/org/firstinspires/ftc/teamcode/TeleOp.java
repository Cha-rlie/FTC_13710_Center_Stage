package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.swervedrive.Robot;
import org.firstinspires.ftc.teamcode.hardware.swervedrive.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    Robot robot;
    GamepadEx driveOp;
    GamepadEx toolOp;

    // Declaring Commands
    //deadband for joysticks
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);


    public boolean willResetIMU = true;

    private Deposit deposit;
    private Intake intake;
    private Hang hang;

    public void init() {
        robot = new Robot(this, false);
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
        if (driveOp.getButton(GamepadKeys.Button.Y)) {
            willResetIMU = false;
        }
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }


    public void loop() {

        // This must be called from the function that loops in the opmode for everything to run
        CommandScheduler.getInstance().run();

        // Update the variables
        if (driveOp.isDown(GamepadKeys.Button.A)) {
            intake.spin();
        } else if (!driveOp.isDown(GamepadKeys.Button.A)) {
            intake.spinFor5Seconds();
        }

        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad2.right_stick_y); //RIGHT joystick

        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1), checkDeadband(joystick2));

//        //uncomment for live tuning of ROT_ADVANTAGE constant
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        //to confirm that joysticks are operating properly
        telemetry.addData("Joystick 1", joystick1);
        telemetry.addData("Joystick 2", joystick2);

        Button transferButton = new GamepadButton(
                toolOp, GamepadKeys.Button.B
        );

        transferButton.whenPressed(new Transfer(deposit));

        if(toolOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {hang.raise();}
        else if(toolOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {hang.lower();}

        if(toolOp.getButton(GamepadKeys.Button.X)) {
            deposit.placing();
        }

        deposit.rotateWristToAngle(deposit.wristPos);

        if (toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            deposit.upSlides();
        } else if (toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            deposit.downSlides();
        } else {
            deposit.powerOffSlides();
        }

        if(hang.hung) {
            hang.Hang.motor.setPower(0.2);
        }

        deposit.manualV4BControl(toolOp.getRightY(), telemetry);

        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
        telemetry.addData("Hang Pos", hang.Hang.getCurrentPosition());
        telemetry.addData("Slide Pos 1: ", deposit.DS1.motor.getCurrentPosition());
        telemetry.addData("Slide Pos 2: ", deposit.DS2.motor.getCurrentPosition());
        telemetry.update();
    }

    //returns zero vector if joystick is within deadband
    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }
}