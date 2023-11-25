package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Generals.SystemFunctionalities.useManualEnable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Systems.OtherSystem;
import org.firstinspires.ftc.teamcode.Systems.ScoringSystem;
import org.firstinspires.ftc.teamcode.Util.SensorEx.HubBulkRead;

@TeleOp(name = "🤯", group = "DRĂCOS")

public class Center_StagETeleOp extends LinearOpMode
        implements Enums.Other, Enums.Swerve, Enums.Scoring, Enums {
    //public CleverSwerve swerve;
    public ScoringSystem system;
    public OtherSystem other;
    public HubBulkRead core;

    private Thread readingThread, swerveThread, otherSystemThread;
    private Object readingLock = new Object();

    private GamepadEx g1,g2;

    private Telemetry dashboardTelemetry;
    private ElapsedTime timerKnowingIfItIsEndgame;
    private final double timeNeedsToPassToReachEndgame = 120000; //ms

    private double startLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private double driveSensitivity = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();

        timerKnowingIfItIsEndgame.reset();
        timerKnowingIfItIsEndgame.startTime();

        dashboardTelemetry.clearAll();

        while(opModeIsActive()) {

            /**Use other subsystems only when endgame starts*/
            if (g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) { other.setState(PullUpPositions.UP); }

            if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) { other.setState(PullUpPositions.HANGING); }



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


            /**Access to outtake actions is necessary*/
            if (system.isManualOuttakeEnabled() || !useManualEnable) {

                if (g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { system.score(Score.HIGH); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { system.score(Score.MID); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { system.score(Score.LOW); }

                    if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { system.getTo(Position.INTERMEDIARY); }

                }

                if ((g2.wasJustPressed(GamepadKeys.Button.A))) {
                    /**Needed for automation of the scoring system. More details in *SystemController* class*/
                    if (system.getNumberOfStoredPixels() == StoredPixels.ONE) { system.setHasGrabbedOnePixel(true); }
                    else { system.setHasGrabbedOnePixel(false); }

                    if (system.getNumberOfStoredPixels() == StoredPixels.TWO) { system.setHasGrabbedTwoPixels(true);  }
                    else { system.setHasGrabbedTwoPixels(false); }

                    system.getTo(Position.COLLECT);
                    system.pixels(PixelActions.GRAB);
                }

                /**SCORE PIXELS BABYYYY*/
                if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                    system.pixels(PixelActions.SCORE);
                    system.setHasGrabbedOnePixel(false);
                    system.setHasGrabbedTwoPixels(false);
                }

            }

            system.manualControlLift(g2.getLeftY());
            updateAll();
        }
    }


    private boolean isEndgame() { return timerKnowingIfItIsEndgame.milliseconds() >= timeNeedsToPassToReachEndgame; }

    private void updateAll() {
        system.update();
        updateTelemetry();
        other.update();
        g2.readButtons();
        g1.readButtons();
    }

    private void Init() {
        //swerve = swerve.getInstance(this, CleverSwerve.Localizers.IMU);
        core = core.getInstance(hardwareMap);
        system = system.getInstance(this, OpMode.TELE_OP);
        other= other.getInstance(this, OtherTimer.NOT_USING_ENDGAME_TIMER);
        initializeThreads();
        startThreads();

        system.resetNumberOfCollectedPixelsToZero();
        system.initializeSystem();

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        timerKnowingIfItIsEndgame = new ElapsedTime();
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboardTelemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
        dashboardTelemetry.update();
    }

    private void initializeThreads() {
        /*swerveThread = new Thread(()-> { while (!isStopRequested()) {
            while(opModeIsActive()) {
            swerve.joystickDrive(
                    g1.getLeftX() * driveSensitivity,
                    -g1.getLeftY() * driveSensitivity,
                    g1.getRightX() * driveSensitivity);

            g1.readButtons();
        }}});*/

        readingThread = new Thread(() -> {
                while (!isStopRequested()) {
                    synchronized (readingLock) {
                        system.read();
                        other.read();
                        core.clearCache(Hubs.ALL);
                }
            }
        });



    }

    private void startThreads() {
        //swerveThread.start();
        readingThread.start();
    }

    private void readAll() {
        system.read();
        core.clearCache(Hubs.ALL);
    }

    private void WaitForStart() { while(!isStarted() && !isStopRequested()) { system.update(); } }

    private void updateTelemetry() {
        double endLoopTime = System.nanoTime();

        dashboardTelemetry.addData("Loop frequency: ", secondsToNanoseconds / (endLoopTime - startLoopTime));

        //dashboardTelemetry.addData("AMPS: ", system.getLiftAmperage());
        dashboardTelemetry.addData("manualIntake: ", system.isManualIntakeEnabled());
        dashboardTelemetry.addData("manualOuttake: ", system.isManualOuttakeEnabled());
        dashboardTelemetry.addData("pixels in possession: ", system.getNumberOfStoredPixels());
        dashboardTelemetry.addData("last collected pixel: ", system.getLastCollectedPixel());

        dashboardTelemetry.addData("lift MAX: ", system.getMAX());
        dashboardTelemetry.addData("lift MIN: ", system.getMIN());
        dashboardTelemetry.addData("lift target: ", system.getTarget());

        //dashboardTelemetry.addData("R: ", system.getR());
        //dashboardTelemetry.addData("G: ", system.getG());
        //dashboardTelemetry.addData("B: ", system.getB());

        //dashboardTelemetry.addData("transfer sensor 1: ", system.getTransferDistance(Sensors.FOR_PIXEL_1, DistanceUnit.CM));
        //dashboardTelemetry.addData("transfer sensor 2: ", system.getTransferDistance(Sensors.FOR_PIXEL_2, DistanceUnit.CM));

        dashboardTelemetry.addData("is first pixel in transfer: ", system.isIsFirstPixelInTransfer());
        dashboardTelemetry.addData("is second pixel in transfer: ", system.isIsSecondPixelInTransfer());
        dashboardTelemetry.addData("get actual number of pixels: ", system.getNumberOfPossessedPixels());

        //dashboardTelemetry.addData("Velocity: ", system.getProfileVelocity());

        startLoopTime = endLoopTime;

        dashboardTelemetry.update();
    }

}
