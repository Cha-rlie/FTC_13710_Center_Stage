package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;

abstract public class OpModeTemplate extends CommandOpMode {
    protected Drivebase drivebase;
    protected Deposit deposit;
    protected Intake intake;
    protected Hang hang;
    protected DroneLauncher shooter;
    protected Lift lift;
    protected IMU imu;

    protected GamepadEx driveOp;
    protected GamepadEx toolOp;

    protected void initHardware(boolean isAuto) {

        // Initialise the imuGyro with the correct orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();


        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        drivebase = new Drivebase(hardwareMap);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap, driveOp);
        hang = new Hang(hardwareMap);
        shooter = new DroneLauncher(hardwareMap);
        lift = new Lift(hardwareMap);
        telemetry.addLine("initialization complete");
        telemetry.update();


        register(intake, drivebase, hang, shooter, deposit);

    }
}