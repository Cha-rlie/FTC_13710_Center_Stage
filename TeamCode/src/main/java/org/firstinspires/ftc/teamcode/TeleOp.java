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
import org.firstinspires.ftc.teamcode.commands.TransferCommand;


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
        new GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(new TransferCommand(deposit, lift, intake));

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


//        lift.run(toolOp.getLeftY());
//        hang.run(gamepad1);

        deposit.manualV4BControl(toolOp.getRightY(), telemetry);
//        deposit.manualSpinControl(toolOp.getLeftY(), telemetry);

        telemetry.addData("Wrist Pos", deposit.Wrist.getAngle());
        telemetry.addData("Spin Pos", deposit.Spin.getAngle());

        telemetry.update();



    }
}