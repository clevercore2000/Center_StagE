package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "RR_Localization",group = "RR")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CleverSwerve swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM);

        waitForStart();

        while (!isStopRequested()) {
            swerve.joystickDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addLine("             POSE"                                       );
            telemetry.addData("x:        ", swerve.getPoseEstimate().x                      );
            telemetry.addData("y:        ", swerve.getPoseEstimate().y                      );
            telemetry.addData("heading:  ", Math.toDegrees(swerve.getPoseEstimate().heading));

            telemetry.update();
        }
    }
}