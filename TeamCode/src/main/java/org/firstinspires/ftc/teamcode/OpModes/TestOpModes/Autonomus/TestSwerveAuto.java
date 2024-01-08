package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

@Config
@Autonomous (name = "Swerve", group = "test")
public class TestSwerveAuto extends LinearOpMode {
    private CleverSwerve swerve;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        CleverData constrains = new CleverData()
                .setFieldCentric(false)
                .add(Enums.Swerve.MotionPackage.PID);

        swerve = new CleverSwerve(this, CleverSwerve.Localizers.ROADRUNNER_TWO_WHEELS, Enums.OpMode.AUTONOMUS);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose startPose = new Pose(0, 0, Math.toRadians(0));
        swerve.setPoseEstimate(startPose);

        /*TrajectorySequence testTrajectory = swerve.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 20, 0))
                .build();*/

        swerve.setTelemetry(dashboardTelemetry);
        double time = 0;

        waitForStart();

        Pose targetPose = new Pose(40, 20, Math.toRadians(0));
        //swerve.followTrajectorySequenceAsync(testTrajectory);

        while (opModeIsActive()) {
            swerve.lockToPosition(targetPose);

            if (swerve.isBusy() && time == 0) { targetPose = new Pose(40, 30, Math.toRadians(180)); time++; }

            dashboardTelemetry.addData("HAS LOCKED: ", !swerve.isBusy());
            dashboardTelemetry.addLine("             POSE"                                       );
            dashboardTelemetry.addData("x:        ", swerve.getPoseEstimate().x                      );
            dashboardTelemetry.addData("y:        ", swerve.getPoseEstimate().y                      );
            dashboardTelemetry.addData("heading:  ", Math.toDegrees(swerve.getPoseEstimate().heading));

            dashboardTelemetry.addLine("             TARGET"                                       );
            dashboardTelemetry.addData("tx:        ", targetPose.x                      );
            dashboardTelemetry.addData("ty:        ", targetPose.y                      );
            dashboardTelemetry.addData("theading:  ", Math.toDegrees(targetPose.heading));

            updateAll();
        }

    }

    private void updateAll() {
        swerve.read();
        swerve.update();
        dashboardTelemetry.update();
    }

}
