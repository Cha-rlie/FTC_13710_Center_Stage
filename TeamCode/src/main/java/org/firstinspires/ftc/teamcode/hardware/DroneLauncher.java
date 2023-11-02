package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DroneLauncher extends SubsystemBase {
    public ServoEx Shoot; // Deposit Slide 1

    public DroneLauncher(HardwareMap hardwareMap) {
        // Assign variables here with parameters
        Shoot = new SimpleServo(hardwareMap, "Shoot", 0, 260, AngleUnit.DEGREES);

        Shoot.turnToAngle(0);
    }

    public void shoot() {
        Shoot.turnToAngle(100);
    }

}
