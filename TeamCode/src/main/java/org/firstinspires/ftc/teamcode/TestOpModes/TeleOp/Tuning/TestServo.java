package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "ConfigureServo", group = "tuning")
public class TestServo extends LinearOpMode {
    private Servo testServo, testServo_right;

    private Telemetry dashboardTelemetry;
    private GamepadEx g1;

    private static List<String> names = Arrays.asList("gripper", "claw", "left_arm", "right_arm", "rotation");
    private static int i = 4, second_i = 3;

    private double position = 0.0;
    private double increment = 0.05;
    private final double incrementSmall = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        g1 = new GamepadEx(gamepad1);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        testServo = hardwareMap.get(Servo.class, names.get(i));
        testServo_right = hardwareMap.get(Servo.class, names.get(second_i));


        waitForStart();

        while (opModeIsActive()) {


            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                position -= increment;
            else if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                position += increment;

            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (increment == incrementSmall)
                    increment = 0.05;
                else increment = incrementSmall;
            }

            testServo.setPosition(position);
            //position = testServo.getPosition();

            dashboardTelemetry.addData("position: ", position);

            dashboardTelemetry.update();
            g1.readButtons();

        }

    }
}
