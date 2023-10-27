package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Hang extends SubsystemBase {

    public MotorEx Hang; // Deposit Slide 1

    public Hang(HardwareMap hardwareMap) {
        // Assign variables here with parameters
        Hang = new MotorEx(hardwareMap, "Hang");
        Hang.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void raise() {
        Hang.motor.setPower(-0.7);
    }

    public void lower() {
        Hang.motor.setPower(0.1);
    }
}
