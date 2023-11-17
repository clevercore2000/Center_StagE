package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Util.Math.MathFormulas.sign;
import static org.firstinspires.ftc.teamcode.Util.Math.MathFormulas.toPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.Systems.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.OtherSystem;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.Util.SensorEx.HubBulkRead;

@TeleOp(name = "🤯", group = "DRĂCOS")

public class Center_StagETeleOp extends LinearOpMode {
    //public CleverSwerve swerve;
    public ScoringSystem system;
    //public OtherSystem other;
    public HubBulkRead core;


    private final double liftSensitivity = 100, driveSensitivity = 0.8;
    private int target;

    private GamepadEx g1,g2;

    private Telemetry dashboardTelemetry;
    private ElapsedTime timerKnowingIfItIsEndgame;
    private final double timeNeedsToPassToReachEndgame = 120000; //ms

    private double startLoopTime;
    private double secondsToNanoseconds = 1000000000;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        timerKnowingIfItIsEndgame.reset();
        timerKnowingIfItIsEndgame.startTime();



        /*new Thread(()-> { while (opModeIsActive()) {
            swerve.joystickDrive(
                    g1.getLeftX() * driveSensitivity,
                    -g1.getLeftY() * driveSensitivity,
                    g1.getRightX() * driveSensitivity);

            g1.readButtons();
        }}).start();*/

        while(opModeIsActive()) {

            /**Use other subsystems only when endgame starts*/
            /*if (isEndgame()) {
                if (g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) { other.setState(Enums.PullUpPositions.UP); }

                if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) { other.setState(Enums.PullUpPositions.HANGING); }

            }*/


            /**Access to intake actions is necessary*/
            if (system.isManualIntakeEnabled()) {
                if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { system.startIntake(Enums.IntakeArmStates.DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.startIntake(Enums.IntakeArmStates.STACK_UP); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.startIntake(Enums.IntakeArmStates.STACK_DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.stopIntake(); }

                    /**mManual SPIT*/
                    if (g2.isDown(GamepadKeys.Button.A)) { system.setState(Enums.IntakeMotorStates.SPIT); }
                }

            }


            /**Access to outtake actions is necessary*/
            if (system.isManualOuttakeEnabled()) {

                if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.score(Enums.Scoring.HIGH); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.score(Enums.Scoring.MID); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.score(Enums.Scoring.LOW); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { system.getTo(Enums.Position.INTERMEDIARY); }

                }

                if ((g2.wasJustPressed(GamepadKeys.Button.A))) {
                    system.getTo(Enums.Position.COLLECT);
                    system.pixels(Enums.PixelActions.GRAB);

                    /**Needed for automation of the scoring system. More details in *SystemController* class*/
                    if (system.getNumberOfStoredPixels() == Enums.StoredPixels.ONE) { system.setHasGrabbedOnePixelAlready(true);}
                    else { system.setHasGrabbedOnePixelAlready(false); }
                }

                /**SCORE PIXELS BABYYYY*/
                if (g2.wasJustPressed(GamepadKeys.Button.B)) { system.pixels(Enums.PixelActions.SCORE); }

            }

            system.manualControlLift(g2.getLeftY());
            updateAll();
        }
    }


    private boolean isEndgame() { return timerKnowingIfItIsEndgame.milliseconds() >= timeNeedsToPassToReachEndgame; }

    private void updateAll() {
        updateTelemetry();
        system.update();
        //other.update();
        g2.readButtons();
        core.clearCache(Enums.Hubs.ALL);
    }

    private void Init() {
        //swerve = swerve.getInstance(this, CleverSwerve.Localizers.IMU);
        system = system.getInstance(this);
        //other= other.getInstance(this);
        core = core.getInstance(hardwareMap);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        timerKnowingIfItIsEndgame = new ElapsedTime();
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() { while(!isStarted() && !isStopRequested()) { system.update(); } }

    private void updateTelemetry() {
        double endLoopTime = System.nanoTime();

        dashboardTelemetry.addData("Loop frequency: ", secondsToNanoseconds / (endLoopTime - startLoopTime));

        //dashboardTelemetry.addData("AMPS: ", system.getLiftAmperage());
       // dashboardTelemetry.addData("manualIntake: ", system.isManualIntakeEnabled());
        //dashboardTelemetry.addData("manualOuttake: ", system.isManualOuttakeEnabled());
        //dashboardTelemetry.addData("pixels in possession: ", system.getNumberOfStoredPixels());

        dashboardTelemetry.addData("R: ", system.getR() / 100);
        dashboardTelemetry.addData("G: ", system.getG() / 100);
        dashboardTelemetry.addData("B: ", system.getB() / 100);

        dashboardTelemetry.addData("Velocity: ", system.getProfileVelocity());

        startLoopTime = endLoopTime;

        dashboardTelemetry.update();
    }

}
