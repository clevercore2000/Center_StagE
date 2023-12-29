package org.firstinspires.ftc.teamcode.DifferentActivities.Molimpiada;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init;

import java.util.ArrayList;
import java.util.List;

@Deprecated
@TeleOp(name = "mOlympics", group = "activities")
public class Manevra extends LinearOpMode {
    private enum Eprubete{
        ONE,
        TWO,
        THREE
    }

    private DcMotorEx motor;
    private double motorSpeed = 0.3, motorSlow = 0.1;
    private List<Eprubeta> eprubete = new ArrayList<>();

    private boolean hasToGoPouring = false;
    private boolean hasToStartMotorSpeed = false;
    private boolean hasToStartMotorSlow = false;
    private boolean allTogether = false;
    private Eprubete currentOne = Eprubete.ONE;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        Init.initializeMotor(motor);

        GamepadEx gamepad = new GamepadEx(gamepad1);

        for (int i = 0; i < 3; i++) { eprubete.add(new Eprubeta(this.hardwareMap, Integer.toString(i+1))); if(i == 0) eprubete.get(i).reverseServo(); eprubete.get(i).Init();}

        waitForStart();

        while (opModeIsActive()) {

            /**Button input*/
            if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { hasToGoPouring = false; }

            if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { hasToGoPouring = true; }

            if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && !allTogether) {
                switch (currentOne) {
                    case ONE: { currentOne = Eprubete.TWO; } break;
                    case TWO: { currentOne = Eprubete.THREE; } break;
                    case THREE: { currentOne = Eprubete.ONE; } break;
                    default: {}
                }
            }

            if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { hasToStartMotorSpeed = !hasToStartMotorSpeed; }
            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { hasToStartMotorSlow = !hasToStartMotorSlow; }


            /**Position-tracking logic*/
            if (hasToGoPouring && allTogether) {
                for (int i = 0; i < 3; i++) { eprubete.get(i).goPouring(); }
            } else if (allTogether) {
                for (int i = 0; i < 3; i++) { eprubete.get(i).goUp(); }
            }

            if (hasToGoPouring && !allTogether) {
                switch (currentOne) {
                    case ONE: { eprubete.get(0).goPouring(); } break;
                    case TWO: { eprubete.get(1).goPouring(); } break;
                    case THREE: { eprubete.get(2).goPouring(); } break;
                    default: {}
                }
            } else if (!allTogether) {
                switch (currentOne) {
                    case ONE: { eprubete.get(0).goUp(); } break;
                    case TWO: { eprubete.get(1).goUp(); } break;
                    case THREE: { eprubete.get(2).goUp(); } break;
                    default: {}
                }
            }

            if (hasToStartMotorSpeed) { startMotor(motorSpeed); }
            else { stopMotor(); }

            if (hasToStartMotorSlow) { startMotor(motorSlow); }
            else { stopMotor(); }


            /**Displaying info...*/
            updateTelemetry();
            gamepad.readButtons();
        }


    }

    private void startMotor(double power) { motor.setPower(power); }

    private void stopMotor() {motor.setPower(0); }

    private void updateTelemetry() {
        telemetry.addData("pouring all together: ", allTogether);

        if (!allTogether) { telemetry.addData("current one: ", currentOne); }
        else { telemetry.addLine("                                    "); }

        telemetry.addData("current position: ", (hasToGoPouring) ? "POURING" : "UP");
        telemetry.addLine("                                                     ");

        telemetry.addData("motor speed: ", (hasToStartMotorSpeed) ? "TRUE" : "FALSE");
        telemetry.addData("motor slow: ", (hasToStartMotorSlow) ? "TRUE" : "FALSE");
        telemetry.update();
    }
}
