package org.firstinspires.ftc.teamcode.OpModes.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.OpModes.CleverMode;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;

@Autonomous(name = "BLUE", group = "main")
public class AutoBlue extends CleverMode {
    private CleverBot robot;
    private Enums.Randomization randomization;
    private Telemetry dashboardTelemetry;

    private Thread liftThread, readingThread;

    @Override
    public void runOpMode() {
        Init();

        WaitForStart();

        WhenStarted();

        AutonomusTasks();

    }

    protected void Init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CleverBot()
                .addConstrains(new CleverData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .add(Enums.Swerve.Localizers.ROADRUNNER)
                        .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DEFAULT)
                        .setAutoOnBlue(true)
                        .setAutoReset(false)
                        .setAutoGetToIntermediary(false)
                        .setFieldCentric(false)
                        .getLoopTime(true)
                        .setUsingAprilTag(false)
                        .setUsingOpenCv(true))
                .addTelemetry(dashboardTelemetry)
                .construct(this);

        InitializeThreads();

        robot.initializeScoring();
        robot.initCompleate();
    }

    @Override
    protected void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            robot.searchForProp();
        }
    }

    protected void InitializeThreads() {
        readingThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.read();
                robot.clearBulkCache();
            }
        });

        liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.updateLift();
            }
        });

        readingThread.start();
        liftThread.start();
    }

    protected void WhenStarted() {
        randomization = robot.getPropPosition();
        robot.closeCamera();
    }

    private void parkWithoutLocalization() {
        ElapsedTime timerToGoPark = new ElapsedTime();
        double secondsToGoPark = 1.3;

        timerToGoPark.startTime();

        while (timerToGoPark.seconds() <= secondsToGoPark) { robot.drive(-1, 0, 0);}
    }

    private void placePixelsOnLine()
    {
        ElapsedTime timer = new ElapsedTime();

    }

    @Override
    protected void AutonomusTasks() {
        parkWithoutLocalization();
    }
}
