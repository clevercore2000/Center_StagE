package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;

@Config
@TeleOp(group = "tuning", name = "PID_swerve")
public class PID_controller4swerve extends LinearOpMode {
    private CleverSwerve swerve;
    private GamepadEx g1;
    public static double p, d;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, Enums.Swerve.Localizers.IMU, Enums.OpMode.TELE_OP);
        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            swerve.setPID(p, 0, d);
            swerve.read();
            swerve.drive(g1.getLeftX(), g1.getLeftY(), g1.getRightX(), SwerveConstants.fastConstrain);
        }
    }
}
