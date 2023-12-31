package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;

@TeleOp(name = "💋", group = "DRĂCOS")
public class TeleOpV2 extends CleverMode {
    private CleverBot robot;

    private Telemetry dashboardTelemetry;

    private Thread readingThread, liftThread, swerveThread, telemetryThread;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        //WhenStarted();

        while (opModeIsActive()) {
            /*robot.getStartingLoopTime();

            robot.read();
            robot.updateAll();
            robot.clearBulkCache();*/

            if (robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) { robot.other.setState(Enums.Other.DronePosition.FIRE); }



            /**Access to intake actions is necessary*/
            if (robot.scoring.isManualIntakeEnabled() || !useManualEnable) {
                if (robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                    if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.DOWN); }

                    if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.STACK_UP); }

                    if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.STACK_DOWN); }


                }

            }

            /**Manual SPIT*/
            if (robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (robot.g2.wasJustPressed(GamepadKeys.Button.A)) { robot.scoring.spitIntake(); }
            }

            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { robot.scoring.stopIntake(); }


            /**Manual lowering*/
            if (robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { robot.scoring.getTo(Enums.Scoring.Position.INTERMEDIARY); }
            }

            /**Access to outtake actions is necessary*/
            if (robot.scoring.isManualOuttakeEnabled() || !useManualEnable) {

                if (robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                        if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP) && robot.scoring.hasGrabbedPixels()) { robot.scoring.score(Enums.Scoring.Score.HIGH); }

                        if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) && robot.scoring.hasGrabbedPixels()) { robot.scoring.score(Enums.Scoring.Score.MID); }

                        if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && robot.scoring.hasGrabbedPixels()) { robot.scoring.score(Enums.Scoring.Score.LOW); }


                    /**Grab pixels*/
                    if ((robot.g2.wasJustPressed(GamepadKeys.Button.A))) { robot.scoring.pixels(Enums.Scoring.PixelActions.COLLECT_GRAB); }

                }

                    /**SCORE PIXELS BABYYYY*/
                    if (robot.g2.wasJustPressed(GamepadKeys.Button.B)) { robot.scoring.pixels(Enums.Scoring.PixelActions.SCORE); }
            }

            robot.scoring.manualControlLift(robot.g2.getLeftY());

            //robot.getEndingLoopTime();

            //robot.debuggingTelemetry();
            //robot.updateTelemetry();

        }

    }

    protected void Init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CleverBot()
                .addConstrains(new CleverData()
                        .add(Enums.OpMode.TELE_OP)
                        .add(Enums.Swerve.Localizers.IMU)
                        .addSwerveSensitivity(GamepadKeys.Trigger.LEFT_TRIGGER)
                        .setUsingVelocityToggle(true)
                        .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DEFAULT)
                        .setAutoReset(false)
                        .setAutoGetToIntermediary(false)
                        .setFieldCentric(false)
                        .allowOtherUsageBeforeEndgame(true)
                        .getLoopTime(true)
                        .setUsingAprilTag(false)
                        .setUsingOpenCv(false)
                        .setMultithreading(true))
                .addTelemetry(dashboardTelemetry)
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        robot.scoring.resetNumberOfCollectedPixelsToZero();

        InitializeThreads();

        robot.initializeScoring();
        robot.initCompleate();
    }


    protected void InitializeThreads() {
        readingThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.read();
                robot.updateAll();
            }
        });

        liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                robot.getStartingLoopTime();

                robot.readLift();
                robot.updateLift();

                robot.getEndingLoopTime();

                robot.debuggingTelemetry();
                robot.updateTelemetry();
            }
        });

        swerveThread = new Thread(() -> {
            while (!isStopRequested()) {
                while (opModeIsActive()) {

                    robot.drive(
                            robot.g1.getLeftX(),
                            robot.g1.getLeftY(),
                            robot.g1.getRightX());
        }}});

        /*telemetryThread = new Thread(() -> {
            boolean opModeJustStarted = false;
            while (!isStopRequested()) {
                robot.debuggingTelemetry();
                robot.updateTelemetry();

                if (opModeJustStarted != opModeIsActive() && opModeIsActive())
                    robot.clearTelemetry();

                opModeJustStarted = opModeIsActive();
            }
        });*/


        readingThread.start();
        liftThread.start();
        swerveThread.start();
        //telemetryThread.start();
    }

    protected void WhenStarted() {
        robot.startEndgameTimer();
        robot.clearTelemetry();
    }

    /*@Override
    protected void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            robot.scoring.read();
            robot.updateLift();
            robot.clearBulkCache();
        }
    }*/
}

