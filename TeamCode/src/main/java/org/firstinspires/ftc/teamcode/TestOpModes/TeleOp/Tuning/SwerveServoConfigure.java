package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "tuning")
public class SwerveServoConfigure extends LinearOpMode {
    private String servoName = "FL_servo";
    private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, servoName);

        while (opModeInInit()) { servo.setPosition(0.0); }
    }
}
