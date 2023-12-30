package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.OpenCV.Camera;

@TeleOp(group = "test", name = "RANDOM")
public class RandomizationTest extends LinearOpMode {
    private Camera camera;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new Camera(this, dashboardTelemetry);

        camera.setPipeline(Enums.Pipelines.DETECTING_PROP);

        while (opModeInInit()) {
            dashboardTelemetry.addData("Randomization: ", camera.getRandomization());
            dashboardTelemetry.update();
        }
    }
}
