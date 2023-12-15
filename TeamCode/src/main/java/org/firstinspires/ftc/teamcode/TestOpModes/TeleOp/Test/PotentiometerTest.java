package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "test")
public class PotentiometerTest extends LinearOpMode {
    private AnalogInput sensor;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(AnalogInput.class, "pot");
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            dashboardTelemetry.addData("Voltage: ", sensor.getVoltage());
            dashboardTelemetry.update();
        }
    }
}
