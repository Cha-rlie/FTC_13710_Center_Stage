package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.swervedrive.Robot;
import org.firstinspires.ftc.teamcode.hardware.swervedrive.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    Robot robot;
    GamepadEx gamePad1;
    GamepadEx gamePad2;

    // Declaring Commands

    //deadband for joysticks
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);

    public boolean willResetIMU = true;

    private Deposit deposit;
    private Intake intake;

    public void init() {
        robot = new Robot(this, false);
        gamePad1 = new GamepadEx(gamepad1);
        gamePad2 = new GamepadEx(gamepad2);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, gamePad1);
        telemetry.addLine("initialization complete");
        telemetry.update();


        CommandScheduler.getInstance();

    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (gamePad1.getButton(GamepadKeys.Button.Y)) {
            willResetIMU = false;
        }
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }


    public void loop() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        Vector2d joystick1 = new Vector2d(gamePad1.getLeftX(), -gamePad1.getLeftY()); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamePad1.getRightX(), -gamePad1.getRightY()); //RIGHT joystick

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



        if(toolOp.getButton(GamepadKeys.Button.X)) {
            deposit.placing();
        } else if (toolOp.getButton(GamepadKeys.Button.B)) {
            deposit.pickUp();
        }


//        if (toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
//            deposit.upSlides();
//        } else if (toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            deposit.downSlides();
//        }

        //deposit.moveSlides(telemetry);


        deposit.manualV4BControl(toolOp.getRightY(), telemetry);

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