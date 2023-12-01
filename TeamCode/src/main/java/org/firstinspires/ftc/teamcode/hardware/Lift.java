package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Lift extends SubsystemBase {

    public Motor leftMotor;
    public Motor rightMotor;
    Telemetry telemetry;
    public static double multiplier = 6;
    public static int top = 1250;

    double target = 0;
    public double liftOffset = 0;

    private PIDFController pidf;
    public static double kP = 0.015;
    public static double kI = 0.001;
    public static double kD = 0.0004;
    public static double kF = 0.001;
    public static boolean disable = false;
    public static double maxSlidePower = 0.5;
    public static double minSlidePower = -0.3;

    private boolean goingDown = false;

    public enum LiftPos{
        UP,
        DOWN
    }
    public Lift(HardwareMap hMap, Telemetry telemetry){
        this.telemetry = telemetry;

        pidf = new PIDFController(kP, kI, kD, kF);

        leftMotor = new MotorEx(hMap,"DS1", Motor.GoBILDA.RPM_1150);
        rightMotor = new MotorEx(hMap, "DS2", Motor.GoBILDA.RPM_1150);
        leftMotor.setInverted(true);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    @Override
    public void periodic(){
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setTolerance(30);
        telemetry.addData("target", target);
        telemetry.addData("current", leftMotor.getCurrentPosition());

        target = Range.clip(target, 0 + liftOffset, 900 + liftOffset);

        double power = pidf.calculate(leftMotor.getCurrentPosition(), target);
        power = Range.clip(power, minSlidePower, maxSlidePower);
        telemetry.addData("power", power);
        if(!disable){
            telemetry.addData("moving slides", power);
            leftMotor.set(power);
            rightMotor.set(power);
        } else{
            leftMotor.set(0);
            rightMotor.set(0);
        }
        telemetry.update();
    }



    public void top(){
        target = top+liftOffset;
        goingDown = false;
    }

    public void bottom(){
        if(target >= 400)
            top = (int) target;
        target = liftOffset+50;
        goingDown = true;
    }



    public void updateTarget(double value){
        target += value * multiplier;
    }

    public double getPosition(){
        return leftMotor.getCurrentPosition();
    }
}