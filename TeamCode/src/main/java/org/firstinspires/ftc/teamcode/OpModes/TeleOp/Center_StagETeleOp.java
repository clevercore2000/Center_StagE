package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.OtherSystem;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

@TeleOp(name = "🤯", group = "DRĂCOS")
@Disabled
public class Center_StagETeleOp extends LinearOpMode
        implements Enums.Other, Enums.Swerve, Enums.Scoring, Enums {
    public CleverSwerve swerve;
    public ScoringSystem system;
    public OtherSystem other;
    //public HubBulkRead core;

    private Thread readingThread, swerveThread, liftThread;
    private Object readingLock = new Object();

    private GamepadEx g1,g2;

    private Telemetry dashboardTelemetry;

    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private double driveSensitivity = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();


        other.start();
        dashboardTelemetry.clearAll();

        while(opModeIsActive()) {

            /**Use other subsystems only when endgame starts*/

            if (g2.wasJustPressed(GamepadKeys.Button.START)) { other.setState(DronePosition.FIRE); }



            /**Access to intake actions is necessary*/
            if (system.isManualIntakeEnabled() || !useManualEnable) {
                if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { system.startIntake(IntakeArmStates.DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.startIntake(IntakeArmStates.STACK_UP); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.startIntake(IntakeArmStates.STACK_DOWN); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.stopIntake(); }


                }

            }

            /**Manual SPIT*/
            if (g2.isDown(GamepadKeys.Button.RIGHT_BUMPER) && g2.isDown(GamepadKeys.Button.A)) { system.spitIntake(); }

            /**Manual lowering*/
            if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { system.getTo(Position.INTERMEDIARY); }
            }

            /**Access to outtake actions is necessary*/
            if (system.isManualOuttakeEnabled() || !useManualEnable) {

                if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.score(Score.HIGH); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.score(Score.MID); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.score(Score.LOW); }

                }

                /**Grab pixels*/
                if ((g2.wasJustPressed(GamepadKeys.Button.A))) { system.pixels(PixelActions.COLLECT_GRAB, false); }

                /**SCORE PIXELS BABYYYY*/
                if (g2.wasJustPressed(GamepadKeys.Button.B)) { system.pixels(PixelActions.SCORE, false); }

            }

            //system.manualControlLift(g2.getLeftY());
            updateAll();
        }
    }


    private void updateAll() {
        //system.update();
        updateTelemetry();
        other.update();
        g2.readButtons();
    }

    private void Init() {
        swerve = new CleverSwerve(this, Localizers.IMU, OpMode.TELE_OP);

        system = new ScoringSystem(this, OpMode.TELE_OP);
        other = new OtherSystem(this);

        //core = core.getInstance(hardwareMap);

        initializeThreads();
        startThreads();

        //not on actual tele-op
        system.resetNumberOfCollectedPixelsToZero();
        system.initializeSystem();

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboardTelemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
        dashboardTelemetry.update();
    }

    private void initializeThreads() {
        swerveThread = new Thread(()-> { while (!isStopRequested()) {
            while(opModeIsActive()) {
            swerve.drive(
                    g1.getLeftX() * driveSensitivity,
                    g1.getLeftY() * driveSensitivity,
                    g1.getRightX() * driveSensitivity,
                    SwerveConstants.fastConstrain);

            g1.readButtons();
        }}});

        liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                system.update();
            }
        });

        /*readingThread = new Thread(() -> {
            while (!isStopRequested()) {
                        swerve.read();
                        system.read();
                        //other.read();
                        //core.clearCache(Hubs.ALL);
            }
        });*/


    }

    private void startThreads() {
        swerveThread.start();
        liftThread.start();

        //readingThread.start();
    }

    private void WaitForStart() { while(!isStarted() && !isStopRequested()) { idle(); } }

    private void updateTelemetry() {
        endLoopTime = System.nanoTime();

        dashboardTelemetry.addData("Loop frequency: ", secondsToNanoseconds / (endLoopTime - startLoopTime));

        /*dashboardTelemetry.addData("AMPS: ", system.getLiftAmperage());
        dashboardTelemetry.addData("manualIntake: ", system.isManualIntakeEnabled());
        dashboardTelemetry.addData("manualOuttake: ", system.isManualOuttakeEnabled());
        dashboardTelemetry.addData("pixels in possession: ", system.getNumberOfStoredPixels());
        dashboardTelemetry.addData("last collected pixel: ", system.getLastCollectedPixel());

        //dashboardTelemetry.addData("lift MAX: ", system.getMAX());
        //dashboardTelemetry.addData("lift MIN: ", system.getMIN());
        dashboardTelemetry.addData("lift target: ", system.getTarget());

        //dashboardTelemetry.addData("R: ", system.getR());
        //dashboardTelemetry.addData("G: ", system.getG());
        //dashboardTelemetry.addData("B: ", system.getB());

        dashboardTelemetry.addData("transfer sensor 1: ", system.getTransferDistance(Sensors.FOR_PIXEL_1, DistanceUnit.CM));
        dashboardTelemetry.addData("transfer sensor 2: ", system.getTransferDistance(Sensors.FOR_PIXEL_2, DistanceUnit.CM));

        dashboardTelemetry.addData("is first pixel in transfer: ", system.isIsFirstPixelInTransfer());
        dashboardTelemetry.addData("is second pixel in transfer: ", system.isIsSecondPixelInTransfer());
        dashboardTelemetry.addData("get actual number of pixels: ", system.getNumberOfPossessedPixels());

        //dashboardTelemetry.addData("Velocity: ", system.getProfileVelocity());*/

        startLoopTime = endLoopTime;

        dashboardTelemetry.update();
    }

}
