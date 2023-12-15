package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Tuning.Configuration;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureMotor", group = "tuning")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;
    private List<String> names = Arrays.asList("FL", "FR", "BL", "BR", "intake", "rightOuttake", "leftOuttake");
    private GamepadEx g1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, names.get(6));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) { motor.setPower(-g1.getLeftY()); }
    }
}
