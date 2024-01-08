package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other.PullUp;

@TeleOp(group = "test")
public class TestPullUp extends LinearOpMode implements Enums.Other {
    private PullUp pullup;
    private GamepadEx g2;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        pullup = new PullUp(this.hardwareMap);
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            pullup.setPower(g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
                    g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));


        }

        //pullup.hasToGetToHang(true);
        //pullup.setState(Enums.PullUpPositions.HANGING);
        //while (opModeIsActive()) { pullup.update(); }
    }
}
