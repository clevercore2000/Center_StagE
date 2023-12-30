package org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Test;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;
import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.MathFormulas.toPower;

import androidx.core.location.GnssStatusCompat;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Util.SensorEx.HubBulkRead;

@TeleOp(name = "JustOuttakeTest", group = "test")
public class OuttakeTest extends LinearOpMode {
    private CleverBot robot;
    private HubBulkRead core;
    private Outtake outtake;

    private GamepadEx g2;

    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private double manualLiftSensitivity = 100;
    private int manualLiftPower = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new CleverBot().construct(this);
        g2 = new GamepadEx(gamepad2);
        core = new HubBulkRead(this.hardwareMap);

        outtake = new Outtake(this);
        //outtake.setRobotInstance(robot);

        waitForStart();

        while (opModeIsActive()) {
            startLoopTime = endLoopTime;

            outtake.read();
            g2.readButtons();

            if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { outtake.setTarget(Enums.Scoring.LiftStates.HIGH); }

                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { outtake.setTarget(Enums.Scoring.LiftStates.MID); }

                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { outtake.setTarget(Enums.Scoring.LiftStates.LOW); }


                /**Grab pixels*/
                /*if ((g2.wasJustPressed(GamepadKeys.Button.A))) { robot.scoring.pixels(Enums.Scoring.PixelActions.COLLECT_GRAB); }

                }*/

                /**SCORE PIXELS BABYYYY*/
                //if (robot.g2.wasJustPressed(GamepadKeys.Button.B)) { robot.scoring.pixels(Enums.Scoring.PixelActions.SCORE); }
            }

            double value = g2.getLeftY();

            if (Math.abs(value) > 0.1) {
                double sign =  (manualLiftPower % 2 == 0) ? Math.signum(value) : 1;
                int target = (int) (outtake.getCurrentPositionAverage() + toPower(value, manualLiftPower) * manualLiftSensitivity * sign);

                outtake.setTarget(target, false);
            }

            outtake.update();
            core.clearCache(Enums.Hubs.ALL);

            endLoopTime = System.nanoTime();
            telemetry.addData("Loop Time: ", secondsToNanoseconds / (endLoopTime - startLoopTime));
            telemetry.update();

        }

    }
}
