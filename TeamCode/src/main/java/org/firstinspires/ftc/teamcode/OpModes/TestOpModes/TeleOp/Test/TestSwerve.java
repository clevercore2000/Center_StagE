package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;

@TeleOp(name = "🥵", group = "test")
public class TestSwerve extends LinearOpMode {
    public CleverSwerve swerve;
    private GamepadEx g1;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        g1 = new GamepadEx(gamepad1);

        swerve = new CleverSwerve(this, CleverSwerve.Localizers.IMU, Enums.OpMode.TELE_OP);
        swerve.setTelemetry(dashboardTelemetry);

        dashboardTelemetry.addLine("INIT FINISHED");
        dashboardTelemetry.update();

        waitForStart();

        dashboardTelemetry.clearAll();
        swerve.resetAngle();

        while (opModeIsActive()) {
            swerve.drive(g1.getLeftX(), g1.getLeftY(), g1.getRightX(), SwerveConstants.fastConstrain);

            if (swerve.getLocalizer() instanceof CustomSwerveLocalizer) {
                dashboardTelemetry.addLine("             POSE");
                dashboardTelemetry.addData("x:        ", swerve.getPoseEstimate().x);
                dashboardTelemetry.addData("y:        ", swerve.getPoseEstimate().y);
                dashboardTelemetry.addData("heading:  ", Math.toDegrees(swerve.getPoseEstimate().heading));
            }

            updateAll();
        }
    }

    private void updateAll() {
        swerve.updateDebuggingTelemetry();
        dashboardTelemetry.update();
        g1.readButtons();
    }
}
