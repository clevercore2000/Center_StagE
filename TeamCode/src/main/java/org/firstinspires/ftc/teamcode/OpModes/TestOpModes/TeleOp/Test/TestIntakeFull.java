package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

@Config
@Deprecated
@TeleOp (name = "FullIntake", group = "test")
public class TestIntakeFull extends LinearOpMode implements Enums.Scoring {
    public ScoringSystem system;

    Telemetry dashboardTelemetry;
    private int target;

    private double startLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private GamepadEx g1,g2;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();


        while(opModeIsActive()) {

            /**Access to intake actions is necessary*/
            /**Access to intake actions is necessary*/
            if (system.isManualIntakeEnabled()) {
                if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.startIntake(IntakeArmStates.DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.startIntake(IntakeArmStates.STACK_UP); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.startIntake(IntakeArmStates.STACK_DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.Y)) { system.stopIntake(); }

                    /**mManual SPIT*/
                    if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) { system.setState(IntakeMotorStates.SPIT); }
                }

            }

            if (g2.wasJustPressed(GamepadKeys.Button.B)) { system.resetNumberOfCollectedPixelsToZero(); }

            system.update();
            g2.readButtons();
            addTelemetry();
        }
    }


    private void Init() {
        system = new ScoringSystem(this, Enums.OpMode.TELE_OP);
        system.resetNumberOfCollectedPixelsToZero();

        dashboardTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        g2 = new GamepadEx(gamepad2);
    }

    private void WaitForStart(){
        while(!opModeIsActive()) { system.update();}
    }


    private void addTelemetry() {
        double endLoopTime = System.nanoTime();

        dashboardTelemetry.addData("Loop frequency: ", secondsToNanoseconds / (endLoopTime - startLoopTime));
        dashboardTelemetry.addData("Last collected pixel", system.getLastCollectedPixel());
        dashboardTelemetry.addData("Number of collected pixels", system.getNumberOfStoredPixels());

        dashboardTelemetry.addLine();

        dashboardTelemetry.addData("R: ", system.getR() / 100);
        dashboardTelemetry.addData("G: ", system.getG() / 100);
        dashboardTelemetry.addData("B: ", system.getB() / 100);

        dashboardTelemetry.addLine();

        startLoopTime = endLoopTime;

        dashboardTelemetry.update();
    }
}
