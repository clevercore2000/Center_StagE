package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Generals.Localizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathBuilders.PurePursuit;

@Autonomous(group = "test", name = "PurePursuit")
public class TestPurePursuit extends LinearOpMode {
    private PurePursuit follower;
    private Localizer localizer;
    private CleverSwerve swerve;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        localizer = new CustomSwerveLocalizer(this.hardwareMap);
        localizer.setPositionEstimate(new Pose(0,0,0));


        swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM, Enums.OpMode.TELE_OP);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //follower = new PurePursuit(localizer.getRobotPosition())
              //  .addPoint(new Point(60, 60))
               // .addPoint(new Point(60, 0))
              //  .addPoint(new Point(0, -20));

        waitForStart();
        follower.start();

        while (opModeIsActive()) {
            try { follower.update(); } catch (NotAPolynomialException e) {}
            swerve.lockToPosition(new Pose(follower.getPointToFollow(), 0));

            dashboardTelemetry.addData("point to follow: ", follower.getPointToFollow());
            dashboardTelemetry.update();
        }
    }
}