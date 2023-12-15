package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Localizer.IMU.Threaded_IMU;
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
    CleverSwerve swerve;
    private Telemetry dashboardTelemetry;
    private Threaded_IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.ROADRUNNER, Enums.OpMode.TELE_OP);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = new Threaded_IMU(this);

        waitForStart();

        while (!isStopRequested()) {
            swerve.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            swerve.read();

            dashboardTelemetry.addLine("             POSE"                                       );
            dashboardTelemetry.addData("x:        ", swerve.getPoseEstimate().x                      );
            dashboardTelemetry.addData("y:        ", swerve.getPoseEstimate().y                      );
            dashboardTelemetry.addData("heading:  ", Math.toDegrees(swerve.getPoseEstimate().heading));
            dashboardTelemetry.addData("Angle: ", imu.getAngle(AngleUnit.DEGREES));

            dashboardTelemetry.update();
        }
    }
}