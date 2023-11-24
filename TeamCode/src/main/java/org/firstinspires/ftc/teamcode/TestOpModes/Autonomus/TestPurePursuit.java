package org.firstinspires.ftc.teamcode.TestOpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.Unnamed.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Point;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Pose;
import org.firstinspires.ftc.teamcode.Unnamed.Pathing.PurePursuit;

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


        swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new PurePursuit(localizer)
                .addPoint(new Point(60, 60))
                .addPoint(new Point(60, 0))
                .addPoint(new Point(0, -20));

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
