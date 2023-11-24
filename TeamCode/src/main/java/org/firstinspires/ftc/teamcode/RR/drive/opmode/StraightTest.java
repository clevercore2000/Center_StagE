package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Pose;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RR_Straight", group = "RR")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CleverSwerve drive = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectoryAsync(trajectory);

        Pose poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.x);
        telemetry.addData("finalY", poseEstimate.y);
        telemetry.addData("finalHeading", poseEstimate.heading);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) ;
    }
}