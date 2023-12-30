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
            pullup.setPower(-g2.getRightY());

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { pullup.setState(PullUpPositions.UP); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { pullup.setState(PullUpPositions.HANGING); }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { pullup.setState(PullUpPositions.DOWN); }

            pullup.update();
            g2.readButtons();
            dashboardTelemetry.addData("state: ", pullup.getState());
            dashboardTelemetry.addData("is pressed: ", pullup.isAllTheWayUp());
            dashboardTelemetry.addData("position: ", pullup.getPosition());
            dashboardTelemetry.update();
        }

        //pullup.hasToGetToHang(true);
        //pullup.setState(Enums.PullUpPositions.HANGING);
        //while (opModeIsActive()) { pullup.update(); }
    }
}
