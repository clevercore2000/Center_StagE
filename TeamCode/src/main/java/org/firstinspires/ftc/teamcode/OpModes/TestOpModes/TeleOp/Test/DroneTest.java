package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other.Drone;

@TeleOp(group = "test")
public class DroneTest extends LinearOpMode implements Enums.Other {
    private Drone drone;
    private GamepadEx g2;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        drone = new Drone(this.hardwareMap);
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboardTelemetry.addLine("PRESS START");
        dashboardTelemetry.update();

        waitForStart();

        dashboardTelemetry.clearAll();

        while (opModeIsActive()) {
            g2.readButtons();

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                drone.setState(DronePosition.FIRE);

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                drone.setState(DronePosition.LOAD);


            dashboardTelemetry.addData("State: ", drone.getState());
            dashboardTelemetry.update();

        }


    }
}
