package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Intake;

@TeleOp(group = "test", name = "JustIntakeTest")
public class IntakeTest extends LinearOpMode {
    private Intake intake;
    private GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(this);
        g2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { startIntake(Enums.Scoring.IntakeArmStates.DOWN); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { startIntake(Enums.Scoring.IntakeArmStates.STACK_UP); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { startIntake(Enums.Scoring.IntakeArmStates.STACK_DOWN); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { stopIntake(); }

            g2.readButtons();
            intake.read();
            //intake.update();
        }
    }

    public void startIntake(Enums.Scoring.IntakeArmStates desiredArmState) {
        intake.setState(desiredArmState);
        intake.setState(Enums.Scoring.IntakeMotorStates.COLLECT);
    }

    public void stopIntake() {
        intake.setState(Enums.Scoring.IntakeArmStates.UP);
        intake.setState(Enums.Scoring.IntakeMotorStates.STOP);
    }

    public void spitIntake() {
        intake.setState(Enums.Scoring.IntakeArmStates.DOWN);
        intake.setState(Enums.Scoring.IntakeMotorStates.SPIT);
    }
}
