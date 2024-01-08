package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverData;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

import lombok.Builder;

@TeleOp(name = "🍪", group = "DRĂCOS")
@Disabled
public class TeleOpKooky extends CommandOpMode {
    private CleverBot robot;
    private Telemetry dashboardTelemetry;

    @Override
    public void initialize() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CleverBot()
                .addConstrains(new CleverData()
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
                        .setMultithreading(true))
                .addTelemetry(dashboardTelemetry)
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        robot.scoring.resetNumberOfCollectedPixelsToZero();

        robot.initializeScoring();
        robot.initCompleate();

        robot.updateTelemetry();

        //stop intake
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStopCommand(robot.scoring));

        //high
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(robot.scoring, Enums.Scoring.Score.HIGH));

        //collect - stack up
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(robot.scoring, Enums.Scoring.IntakeArmStates.STACK_UP));

        //mid
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(robot.scoring, Enums.Scoring.Score.MID));

        //collect - stack down
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(robot.scoring, Enums.Scoring.IntakeArmStates.STACK_DOWN));

        //low
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeScoreCommand(robot.scoring, Enums.Scoring.Score.LOW));

        //collect - down
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeStartCommand(robot.scoring, Enums.Scoring.IntakeArmStates.DOWN));

        //intermediary
        robot.g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakeGetToCommand(robot.scoring, Enums.Scoring.Position.INTERMEDIARY));

        //spit
        robot.g2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new IntakeSpitCommand(robot.scoring));

        //grab
        robot.g2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)))
                .whenActive(new OuttakePixelCommand(robot.scoring, Enums.Scoring.PixelActions.COLLECT_GRAB));

        //score pixels bby
        robot.g2.getGamepadButton(GamepadKeys.Button.B)
                .whenActive(new OuttakePixelCommand(robot.scoring, Enums.Scoring.PixelActions.SCORE));

        while (opModeInInit()) {
            robot.read();
            robot.updateAll();
        }

        robot.clearTelemetry();

    }

    @Override
    public void run() {
        super.run();
        robot.getStartingLoopTime();

        robot.read();

        //robot.scoring.manualControlLift(robot.g2.getLeftY());
        robot.updateAll();

        robot.debuggingTelemetry();

        robot.getEndingLoopTime();
        robot.updateTelemetry();
    }
}
