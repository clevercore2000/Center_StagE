package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.useManualEnable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.OpModes.CleverMode;
import org.firstinspires.ftc.teamcode.Systems.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.Systems.Robot.CleverData;

@TeleOp(name = "💋", group = "DRĂCOS")
public class TeleOpV2 extends CleverMode {
    private CleverBot robot;

    private GamepadEx g1, g2;
    private Telemetry dashboardTelemetry;

    private Thread readingThread, liftThread, swerveThread;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        WhenStarted();

        while (opModeIsActive()) {
            g2.readButtons();

            /**Use other subsystems only when endgame starts*/
            if (g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) { robot.other.setState(Enums.Other.PullUpPositions.UP); }

            if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) { robot.other.setState(Enums.Other.PullUpPositions.HANGING); }

            if (g2.wasJustPressed(GamepadKeys.Button.START)) { robot.other.setState(Enums.Other.DronePosition.FIRE); }



            /**Access to intake actions is necessary*/
            if (robot.scoring.isManualIntakeEnabled() || !useManualEnable) {
                if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.STACK_UP); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { robot.scoring.startIntake(Enums.Scoring.IntakeArmStates.STACK_DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { robot.scoring.stopIntake(); }


                }

            }

            /**Manual SPIT*/
            if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (g2.wasJustPressed(GamepadKeys.Button.A)) { robot.scoring.spitIntake(); }
            }

            /**Manual lowering*/
            if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { robot.scoring.getTo(Enums.Scoring.Position.INTERMEDIARY); }
            }

            /**Access to outtake actions is necessary*/
            if (robot.scoring.isManualOuttakeEnabled() || !useManualEnable) {

                if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { robot.scoring.score(Enums.Scoring.Score.HIGH); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { robot.scoring.score(Enums.Scoring.Score.MID); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { robot.scoring.score(Enums.Scoring.Score.LOW); }

                    /**Grab pixels*/
                    if ((g2.wasJustPressed(GamepadKeys.Button.A))) { robot.scoring.pixels(Enums.Scoring.PixelActions.COLLECT_GRAB); }

                }

                /**SCORE PIXELS BABYYYY*/
                if (g2.wasJustPressed(GamepadKeys.Button.B)) { robot.scoring.pixels(Enums.Scoring.PixelActions.SCORE); }

            }

            robot.scoring.manualControlLift(g2.getLeftY());
            robot.updatePullUp();

            robot.addTelemetry("Angle: ", robot.getRobotPosition().heading);
            robot.updateTelemetry();
        }

    }

    protected void Init() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CleverBot()
                .addConstrains(new CleverData()
                        .add(Enums.OpMode.TELE_OP)
                        .add(Enums.Swerve.Localizers.IMU)
                        .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.X_SHAPE)
                        .setAutoReset(false)
                        .setAutoGetToIntermediary(false)
                        .setFieldCentric(false)
                        .allowOtherUsageBeforeEndgame(false)
                        .getLoopTime(true)
                        .setUsingAprilTag(false)
                        .setUsingOpenCv(false))
                .addTelemetry(dashboardTelemetry)
                .construct(this);

        InitializeThreads();

        robot.initializeScoring();
        robot.initCompleate();
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

        swerveThread = new Thread(() -> {
            while (!isStopRequested()) {
                while (opModeIsActive()) {
                    g1.readButtons();

                    robot.drive(
                            g1.getLeftX(),
                            g1.getLeftY(),
                            g1.getRightX());
        }}});

        readingThread.start();
        liftThread.start();
        swerveThread.start();
    }

    protected void WhenStarted() {
        robot.startEndgameTimer();
        robot.clearTelemetry();
    }
}
