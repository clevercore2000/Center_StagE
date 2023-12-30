package org.firstinspires.ftc.teamcode.motion.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RR_Straight", group = "RR")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    CleverSwerve swerve;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM, Enums.OpMode.AUTONOMUS);

        TrajectorySequence trajectory = swerve.trajectorySequenceBuilder(new Pose())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        swerve.followTrajectorySequenceAsync(trajectory);

        Pose poseEstimate = swerve.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.x);
        telemetry.addData("finalY", poseEstimate.y);
        telemetry.addData("finalHeading", poseEstimate.heading);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive() && swerve.isBusy()) ;
    }
}