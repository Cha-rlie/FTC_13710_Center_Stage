package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Import local FTCLib hardware classes
import org.firstinspires.ftc.teamcode.commands.FlattenCommand;
import org.firstinspires.ftc.teamcode.commands.HomeCommand;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends OpModeTemplate {

    public boolean willResetIMU = true;
    ElapsedTime timer = new ElapsedTime();
    boolean commandRun;
    boolean IntakeOneTime = false;

    // Declaring Commands


    @Override
    public void initialize() {
        initHardware(false);

        commandRun = false;

        new GamepadButton(toolOp, GamepadKeys.Button.START).toggleWhenPressed(() -> intake.openCover(), () -> intake.closeCover());
        new GamepadButton(toolOp, GamepadKeys.Button.A).toggleWhenPressed(() -> deposit.grab(), () -> deposit.release());
        new GamepadButton(toolOp, GamepadKeys.Button.BACK).toggleWhenPressed(() -> shooter.shoot(),  () -> shooter.reset());

//        new GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(() -> deposit.place());
        new GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(new HomeCommand(deposit, lift));

        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> deposit.mosaicSpin(1, telemetry)).whenReleased(() -> deposit.mosaicSpin(0, telemetry));
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> deposit.mosaicSpin(-1, telemetry)).whenReleased(() -> deposit.mosaicSpin(0, telemetry));

//        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(() -> deposit.manualWristControl(1, telemetry));
//        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> deposit.manualWristControl(-1, telemetry));

        new TriggerReader(toolOp, GamepadKeys.Trigger.RIGHT_TRIGGER).isDown();

    }

    @Override
    public void run() {
        super.run();
        drivebase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Reset the imu if the driver deems it necessary
        if (gamepad1.guide) {
            imu.resetYaw();
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
            }
        }

//        lift.run(toolOp.getLeftY());
//        hang.run(gamepad2);


        deposit.manualV4BControl(-toolOp.getRightY(), telemetry);

        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());

        telemetry.update();



    }
}