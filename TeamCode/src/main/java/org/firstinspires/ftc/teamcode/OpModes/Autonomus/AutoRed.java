package org.firstinspires.ftc.teamcode.OpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.CleverMode;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

@Autonomous(name = "RED")
public class AutoRed extends CleverMode {
    private CleverBot robot;
    private Enums.Randomization randomization;
    private Telemetry dashboardTelemetry;

    private Thread liftThread, readingThread;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        WhenStarted();

        AutonomusTasks();

        boolean hasGotBack = false;
        while (opModeIsActive()) {
            idle();

            robot.swerve.lockToPosition(new Pose(40, 0, Math.toRadians(180)));

            dashboardTelemetry.addLine("             POSE"                                       );
            dashboardTelemetry.addData("x:        ", robot.swerve.getPoseEstimate().x                      );
            dashboardTelemetry.addData("y:        ", robot.swerve.getPoseEstimate().y                      );
            dashboardTelemetry.addData("heading:  ", Math.toDegrees(robot.swerve.getPoseEstimate().heading));

            dashboardTelemetry.update();
            /*if (!robot.swerve.isBusy() && !hasGotBack) {

                TrajectorySequence goBack = robot.swerve.trajectorySequenceBuilder(robot.getRobotPosition())
                        .lineToLinearHeading(new Pose2d(0,0,0))
                        .build();

                hasGotBack = true;

                robot.swerve.followTrajectorySequenceAsync(goBack);
            }*/
        }

    }

    protected void Init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CleverBot()
                .addConstrains(new CleverData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .add(Enums.Swerve.Localizers.ROADRUNNER)
                        .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DEFAULT)
                        .setAutoOnBlue(false)
                        .setAutoReset(false)
                        .setAutoGetToIntermediary(false)
                        .setFieldCentric(false)
                        .getLoopTime(true)
                        .setUsingAprilTag(false)
                        .setUsingOpenCv(true))
                .addTelemetry(dashboardTelemetry)
                .construct(this);

        robot.initializeScoring();
        robot.setPipeline(Enums.Pipelines.DETECTING_PROP);

        InitializeThreads();

        robot.initCompleate();
    }

    @Override
    protected void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            robot.searchForProp();

            dashboardTelemetry.addData("Randomization: ", robot.getPropPosition());
            dashboardTelemetry.update();
        }
    }

    protected void InitializeThreads() {
        readingThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.read();
                robot.updateSwerve();
                //robot.clearBulkCache();
            }
        });

        liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.readLift();
                robot.updateLift();
            }
        });



        readingThread.start();
        liftThread.start();
    }

    protected void WhenStarted() {
        robot.clearTelemetry();

        randomization = robot.getPropPosition();
        robot.closeCamera();
    }

    private void goToRandomizationPlace() {
        TrajectorySequence goToRandom = null;

        switch (randomization) {
            case LEFT: { goToRandom = robot.swerve.trajectorySequenceBuilder(new Pose())
                    .lineToLinearHeading(new Pose2d(-20, 0 , Math.toDegrees(120)))
                    .waitSeconds(3)
                    .lineToLinearHeading(new Pose2d(0, 0, 0))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(20, 20), 0)
                    .build();
            }
        }

        if (goToRandom != null)
            robot.swerve.followTrajectorySequenceAsync(goToRandom);

    }

    @Override
    protected void AutonomusTasks() {
        //goToRandomizationPlace();
    }
}
