package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

@Config
@Autonomous (name = "Swerve", group = "test")
public class TestSwerveAuto extends LinearOpMode {
    private CleverSwerve swerve;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM, Enums.OpMode.AUTONOMUS);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose startPose = new Pose(0, 0, Math.toRadians(45));
        swerve.setPoseEstimate(startPose);

        TrajectorySequence testTrajectory = swerve.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 40, 90))
                .build();

        waitForStart();

        swerve.followTrajectorySequenceAsync(testTrajectory);

        while (opModeIsActive() && swerve.isBusy()) {
            dashboardTelemetry.addLine("             POSE"                                       );
            dashboardTelemetry.addData("x:        ", swerve.getPoseEstimate().x                      );
            dashboardTelemetry.addData("y:        ", swerve.getPoseEstimate().y                      );
            dashboardTelemetry.addData("heading:  ", Math.toDegrees(swerve.getPoseEstimate().heading));

            updateAll();
        }

    }

    private void updateAll() {
        swerve.update();
        dashboardTelemetry.update();
    }

}