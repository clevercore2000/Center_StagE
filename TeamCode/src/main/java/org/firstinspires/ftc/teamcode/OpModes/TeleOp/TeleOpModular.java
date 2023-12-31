package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Commands.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.OpModes.Commands.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.OpModes.Commands.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.OpModes.Commands.OuttakeGetToCommand;
import org.firstinspires.ftc.teamcode.OpModes.Commands.OuttakePixelCommand;
import org.firstinspires.ftc.teamcode.OpModes.Commands.OuttakeScoreCommand;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.OtherSystem;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Util.SensorEx.HubBulkRead;

@TeleOp(name = ":(", group = "DRĂCOS")
@Disabled
public class TeleOpModular extends CleverMode {
    private ScoringSystem scoring;
    private OtherSystem other;
    private CleverSwerve swerve;
    private HubBulkRead core;

    private GamepadEx g1;
    private GamepadEx g2;
    private Telemetry dashboardTelemetry;

    private CleverData constrains;

    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    @Override
    protected void Init() {
        scoring = new ScoringSystem(this, Enums.OpMode.TELE_OP);
        other = new OtherSystem(this);
        swerve = new CleverSwerve(this, Enums.Swerve.Localizers.IMU, Enums.OpMode.TELE_OP);
        core = new HubBulkRead(this.hardwareMap);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        constrains = new CleverData()
                .add(Enums.OpMode.TELE_OP)
                .add(Enums.Swerve.Localizers.IMU)
                .addSwerveSensitivity(GamepadKeys.Trigger.LEFT_TRIGGER)
                .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DEFAULT)
                .setAutoReset(false)
                .setAutoGetToIntermediary(false)
                .setFieldCentric(false)
                .allowOtherUsageBeforeEndgame(true)
                .getLoopTime(true)
                .setUsingAprilTag(false)
                .setUsingOpenCv(false)
                .setMultithreading(false);

        scoring.resetNumberOfCollectedPixelsToZero();
        scoring.initializeSystem();



        //stop intake
        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStopCommand(scoring));

        //high
        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(scoring, Enums.Scoring.Score.HIGH));

        //collect - stack up
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(scoring, Enums.Scoring.IntakeArmStates.STACK_UP));

        //mid
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(scoring, Enums.Scoring.Score.MID));

        //collect - stack down
        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(scoring, Enums.Scoring.IntakeArmStates.STACK_DOWN));

        //low
        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(scoring, Enums.Scoring.Score.LOW));

        //collect - down
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(scoring, Enums.Scoring.IntakeArmStates.DOWN));

        //intermediary
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeGetToCommand(scoring, Enums.Scoring.Position.INTERMEDIARY));

        //spit
        g2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeSpitCommand(scoring));

        //grab
        g2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakePixelCommand(scoring, Enums.Scoring.PixelActions.COLLECT_GRAB));

        //score pixels bby
        g2.getGamepadButton(GamepadKeys.Button.B)
                .whenActive(new OuttakePixelCommand(scoring, Enums.Scoring.PixelActions.SCORE));


        telemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
        telemetry.update();
    }

    @Override
    protected void WhenStarted() {
        other.start();
        telemetry.clearAll();
    }

    @Override
    protected void InitializeThreads() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        WhenStarted();

        while (opModeIsActive()) {
            startLoopTime = endLoopTime;

            swerve.read();
            other.read();
            scoring.read();
            g1.readButtons();
            g2.readButtons();

            swerve.update();
            scoring.update();
            other.update();
            scoring.manualControlLift(g2.getLeftY());

            core.clearCache(Enums.Hubs.ALL);

            endLoopTime = System.nanoTime();
            telemetry.addData("Loop Time: ", secondsToNanoseconds / (endLoopTime - startLoopTime));
            telemetry.update();
        }
    }
}
