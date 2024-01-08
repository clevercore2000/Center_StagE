package org.firstinspires.ftc.teamcode.motion.RR.drive.opmode;

import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.Transformations.Pose2d_2_Pose;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(name = "RR_Back&Forth", group = "RR")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;
    CleverSwerve swerve;
    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.ROADRUNNER_THREE_WHEELS, Enums.OpMode.AUTONOMUS);

        TrajectorySequence trajectoryForward = swerve.trajectorySequenceBuilder(new Pose())
                .forward(DISTANCE)
                .build();

        TrajectorySequence trajectoryBackward = swerve.trajectorySequenceBuilder(Pose2d_2_Pose(trajectoryForward.end()))
                .back(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            swerve.followTrajectorySequenceAsync(trajectoryForward);
            swerve.followTrajectorySequenceAsync(trajectoryBackward);
        }
    }
}