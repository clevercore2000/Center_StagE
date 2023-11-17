package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring.ScoringSystem;

@Deprecated
@TeleOp(group = "test", name = "IntakeNoSensors")
public class TestIntakeNoSensors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ScoringSystem intake = new ScoringSystem(this);
        GamepadEx g1 = new GamepadEx(gamepad1);
        GamepadEx g2 = new GamepadEx(gamepad2);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) { intake.update(); }
        }).start();

        while (opModeIsActive()) {
            if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER) && g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { intake.setState(Enums.IntakeMotorStates.SPIT); }
            
            if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { intake.setState(Enums.IntakeArmStates.UP); }

            if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { intake.setState(Enums.IntakeArmStates.DOWN); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                intake.setState(Enums.IntakeArmStates.UP);
                intake.setState(Enums.IntakeMotorStates.STOP);
            }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                intake.setState(Enums.IntakeArmStates.DOWN);
                intake.setState(Enums.IntakeMotorStates.COLLECT);
            }
        }
    }
}
