package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

@TeleOp(name = "PixelDetection",  group = "test")
public class TestPixelDetection extends LinearOpMode {
    Camera camera;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new Camera(this, dashboardTelemetry);
        camera.setPipeline(Enums.Pipelines.DETECTING_PIXELS_ON_BACKDROP);


        waitForStart();
    }
}
