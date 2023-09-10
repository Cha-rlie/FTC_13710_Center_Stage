package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//INSTRUCTIONS:
// align modules to be facing the same direction (make sure not 180 degrees apart)
// press y button on controller 1 (configured with start+A)
// make sure telemetry encoder values are both 0
// if robot controls are inverted, repeat this process, but turn both modules 180 degrees away from where you reset them before

//MUST be run every time program is downloaded
//does NOT have to be run before every TeleOp/Auto run
//probably should be used to verify that encoders have not drifted before every competition match

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends OpMode {
    Robot robot;
    boolean wheel1Reset = false;
    boolean wheel2Reset = false;

    public void init () {
        robot = new Robot(this, false);
        robot.initIMU();
    }

    public void loop () {
        telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();

        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick

        if(!wheel1Reset && !wheel2Reset) {
            robot.driveController.moduleLeft.rotateModule(joystick1, false);
        } else if (!wheel2Reset && wheel1Reset) {
            robot.driveController.moduleRight.rotateModule(joystick1, false);
        }

        if(gamepad1.a) {
            wheel1Reset = true;
        } else if (gamepad1.b) {
            wheel2Reset = true;
        }


        if (gamepad1.y) {
            robot.driveController.resetEncoders();
        }
    }
}
