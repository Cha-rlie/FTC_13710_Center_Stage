package org.firstinspires.ftc.teamcode.teleoop;

// Imports for the FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Imports for FTCLib
import com.arcrobotics.ftclib.hardware.motors.Motor;

@TeleOp

public class Simples extends LinearOpMode {
//    DcMotor motor1;
//    DcMotor motor2;

    public void runOpMode() {
        //motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        Motor motor1 = new Motor(hardwareMap, "motor1");
        Motor motor2 = new Motor(hardwareMap, "motor2");
        // motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.resetEncoder();
        motor2.resetEncoder();
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor1.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setInverted(true);

        waitForStart();

        while(opModeIsActive()) {
            //motor1.setPower(0.4);
            motor1.set(0.4);
            motor2.set(0.2);

            telemetry.addData("Motor 1 Encoder: ", motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Encoder: ", motor2.getCurrentPosition());
        }
    }

}
