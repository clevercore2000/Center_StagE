package org.firstinspires.ftc.teamcode.hardware.Robot;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.fastConstrain;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.slowConstrain;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.usingButtonSensitivity;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.usingDriveSensitivity;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.usingVelocityToggle;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.usingOpenCvCamera;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.OtherSystem;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Util.SensorEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

import javax.annotation.Nullable;

public class CleverBot {
    public ScoringSystem scoring;
    public OtherSystem other;
    public CleverSwerve swerve;

    public GamepadEx g1, g2;
    private boolean add_g1, add_g2;

    private boolean override = false, overrideRotation = false;

    private VoltageSensor batteryVoltageSensor;
    public static double batteryVoltage;
    private HubBulkRead core;
    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private double speed;
    private boolean isFast = true;
    private Trigger isGamepadTriggerPressed;
    private boolean lastTriggerValue = false;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;
    private Telemetry telemetry;

    //default constrains
    private CleverData constrains = new CleverData()
            .add(Enums.Swerve.Localizers.ROADRUNNER_THREE_WHEELS)
            .add(Enums.Swerve.MotionPackage.CUSTOM)
            .add(Enums.OpMode.TELE_OP)
            .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DIAMOND)
            .allowOtherUsageBeforeEndgame(false)
            .setUsingAprilTag(false)
            .setUsingOpenCv(false);



    /**__________/\\\\\\\\\__/\\\\\\________________________________________________________________
     _______/\\\////////__\////\\\__________________________________________________________________
     _____/\\\/______________\/\\\__________________________________________________________________
     ____/\\\________________\/\\\________/\\\\\\\\___/\\\____/\\\_____/\\\\\\\\___/\\/\\\\\\\______
     ___\/\\\________________\/\\\______/\\\/////\\\_\//\\\__/\\\____/\\\/////\\\_\/\\\/////\\\_____
     ___\//\\\_______________\/\\\_____/\\\\\\\\\\\___\//\\\/\\\____/\\\\\\\\\\\__\/\\\___\///______
     ____\///\\\_____________\/\\\____\//\\///////_____\//\\\\\____\//\\///////___\/\\\_____________
     ______\////\\\\\\\\\__/\\\\\\\\\__\//\\\\\\\\\\____\//\\\______\//\\\\\\\\\\_\/\\\_____________
     _________\/////////__\/////////____\//////////______\///________\//////////__\///______________*/



    public CleverBot addConstrains(CleverData constrains) {
        this.constrains = constrains;
        return this;
    }

    public CleverBot addTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        return this;
    }

    public CleverBot addGamepads(Enums.Gamepads gamepads) {
        switch (gamepads) {
            case G1: {
                add_g1 = true;
                add_g2 = false;
            } break;

            case G2: {
                add_g1 = false;
                add_g2 = true;
            } break;

            case BOTH: {
                add_g1 = true;
                add_g2 = true;
            } break;
        }

        return this;
    }

    public CleverBot construct(LinearOpMode opMode) {
        this.opMode = opMode;

        scoring = new ScoringSystem(opMode, constrains.opModeType);
        other = new OtherSystem(opMode);
        swerve = new CleverSwerve(opMode, constrains.localizer, constrains.opModeType);

        g1 = add_g1 ? new GamepadEx(opMode.gamepad1) : null;
        g2 = add_g2 ? new GamepadEx(opMode.gamepad2) : null;

        openCvCamera = usingOpenCvCamera ? new Camera(opMode, telemetry) : null;
        aprilTagCamera = usingAprilTagCamera ? new AprilTagCamera(opMode, telemetry) : null;

        isGamepadTriggerPressed = !usingButtonSensitivity && usingVelocityToggle ?
                new Trigger(() -> g1.getTrigger(constrains.sensitivityTrigger) > 0.01) : null;

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        //core = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.AUTO);

        if (constrains.sensitivityTrigger == null && constrains.sensitivityButton == null)
            usingDriveSensitivity = false;

        swerve.setBatteryVoltageSensor(batteryVoltageSensor);

        setRobotInstance();

        return this;
    }



    /**________________________/\\\\\\\\\___________________________________________________________
     _____________________/\\\////////______________________________________________________________
     ___________________/\\\/_______________________________________________________________________
     __________________/\\\_________________/\\\\\_____/\\/\\\\\\\______/\\\\\\\\___________________
     _________________\/\\\_______________/\\\///\\\__\/\\\/////\\\___/\\\/////\\\__________________
     _________________\//\\\_____________/\\\__\//\\\_\/\\\___\///___/\\\\\\\\\\\___________________
     __________________\///\\\__________\//\\\__/\\\__\/\\\_________\//\\///////____________________
     ____________________\////\\\\\\\\\__\///\\\\\/___\/\\\__________\//\\\\\\\\\\__________________
     _______________________\/////////_____\/////_____\///____________\//////////___________________*/










    /**__________/\\\\\\\\\__/\\\\\\________________________________________________________________
     _______/\\\////////__\////\\\__________________________________________________________________
     _____/\\\/______________\/\\\__________________________________________________________________
     ____/\\\________________\/\\\________/\\\\\\\\___/\\\____/\\\_____/\\\\\\\\___/\\/\\\\\\\______
     ___\/\\\________________\/\\\______/\\\/////\\\_\//\\\__/\\\____/\\\/////\\\_\/\\\/////\\\_____
     ___\//\\\_______________\/\\\_____/\\\\\\\\\\\___\//\\\/\\\____/\\\\\\\\\\\__\/\\\___\///______
     ____\///\\\_____________\/\\\____\//\\///////_____\//\\\\\____\//\\///////___\/\\\_____________
     ______\////\\\\\\\\\__/\\\\\\\\\__\//\\\\\\\\\\____\//\\\______\//\\\\\\\\\\_\/\\\_____________
     _________\/////////__\/////////____\//////////______\///________\//////////__\///______________*/



    public void read() {
        batteryVoltage = batteryVoltageSensor.getVoltage();

        scoring.read();
        swerve.read();

        if (g1 != null) g1.readButtons();
        if (g2 != null) g2.readButtons();

    }

    public void gamepadRumble(Gamepad gamepad, Enums.Rumbles type) {
        switch (type) {
            case START_PULLUP: {
                Gamepad.RumbleEffect start = new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, 1.0, 300)
                        .addStep(1.0, 0.0, 300)
                        .build();

                gamepad.setLedColor(128, 0, 128, 1000); //purple
                gamepad.runRumbleEffect(start);
            } break;

            case STOP_PULLUP: {
                Gamepad.RumbleEffect stop = new Gamepad.RumbleEffect.Builder()
                        .addStep(1.0, 1.0, 300)
                        .build();

                gamepad.setLedColor(255, 165, 0, 1000); //orange
                gamepad.runRumbleEffect(stop);
            } break;

            case FAST_SWERVE: {} break;

            case SLOW_SWERVE: {
                gamepad.setLedColor(128, 0, 128, 1000); //purple again
            } break;
        }
    }

    public void manualControlLift(double value) {

        if (g2.isDown(GamepadKeys.Button.X) && g2.isDown(GamepadKeys.Button.Y)) {
            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                override = !override;
                overrideRotation = !overrideRotation;

                if (overrideRotation) { gamepadRumble(g2.gamepad, Enums.Rumbles.START_PULLUP); }
                    else { gamepadRumble(g2.gamepad, Enums.Rumbles.STOP_PULLUP); }
            }
        }

        scoring.manualControlLift(value, override, overrideRotation);

    }

    public void readLift() {
        scoring.readLift();
    }

    public void updateLift() { scoring.updateLift();}

    public void updateSwerve() { swerve.update(); }

    public void updateAll() {
        scoring.update();
        swerve.update();
    }



    /**________________________/\\\\\\\\\___________________________________________________________
     _____________________/\\\////////______________________________________________________________
     ___________________/\\\/_______________________________________________________________________
     __________________/\\\_________________/\\\\\_____/\\/\\\\\\\______/\\\\\\\\___________________
     _________________\/\\\_______________/\\\///\\\__\/\\\/////\\\___/\\\/////\\\__________________
     _________________\//\\\_____________/\\\__\//\\\_\/\\\___\///___/\\\\\\\\\\\___________________
     __________________\///\\\__________\//\\\__/\\\__\/\\\_________\//\\///////____________________
     ____________________\////\\\\\\\\\__\///\\\\\/___\/\\\__________\//\\\\\\\\\\__________________
     _______________________\/////////_____\/////_____\///____________\//////////___________________*/



    public void clearBulkCache() { core.clearCache(Enums.Hubs.ALL); }

    public Pose getRobotPosition() { return swerve.getPoseEstimate(); }

    public double getAngle() { return swerve.getAngle(); }

    public double getBatteryVoltage() { return batteryVoltage; }

    public void setRobotPosition(Pose newPose) { swerve.setPoseEstimate(newPose); }

    public void setRobotInstance() {
        scoring.setRobotInstance(this);
        other.setRobotInstance(this);
        swerve.setRobotInstance(this);
    }

    public void setPullUpPower(double power) { other.setPower(power);}

    public void startEndgameTimer() { other.start(); }

    public void initializeScoring() { scoring.initializeSystem(); }

    public void drive(double x, double y, double heading) {

        if (usingDriveSensitivity) {
            if (usingVelocityToggle) {
                if (usingButtonSensitivity && g1.wasJustPressed(constrains.sensitivityButton))
                    isFast = !isFast;
                else if(!usingButtonSensitivity) {

                    if (isGamepadTriggerPressed.get() && !lastTriggerValue)
                        isFast = !isFast;
                    lastTriggerValue = isGamepadTriggerPressed.get();

                }
                speed = isFast ? fastConstrain : slowConstrain;

            }
            else {
                speed = fastConstrain;

                if (usingButtonSensitivity && g1.isDown(constrains.sensitivityButton))
                    speed = slowConstrain;
                else if (!usingButtonSensitivity && g1.getTrigger(constrains.sensitivityTrigger) > 0.01)
                    speed = slowConstrain;
            }
        }

        if (speed == slowConstrain) { gamepadRumble(g1.gamepad, Enums.Rumbles.SLOW_SWERVE); }

        swerve.drive(x, y, heading, speed);
    }

    public void getStartingLoopTime() { startLoopTime = endLoopTime; }

    public void getEndingLoopTime() { endLoopTime = System.nanoTime(); }

    public double getLoopFrequency() { return telemetryAddLoopTime ? secondsToNanoseconds / (endLoopTime - startLoopTime) : 0; }

    public double getClosestAprilTag() { return (aprilTagCamera == null) ? 0 : aprilTagCamera.getDetection(); }

    public void searchForProp() {
        if (openCvCamera == null || !(openCvCamera.getPipeline() instanceof PropDetectionPipeline)) { randomization = null; }
            else {
                Enums.Randomization detection = openCvCamera.getRandomization();

                if (detection != null)
                    randomization = detection;
        }
    }

    @Nullable
    public Enums.Randomization getPropPosition() { return (usingOpenCvCamera) ? randomization : null; }

    public void setPipeline(Enums.Pipelines pipeline) { if (usingOpenCvCamera) openCvCamera.setPipeline(pipeline); }

    public void closeCamera() { if (usingOpenCvCamera) openCvCamera.close(); }



    /**__________/\\\\\\\\\__/\\\\\\________________________________________________________________
     _______/\\\////////__\////\\\__________________________________________________________________
     _____/\\\/______________\/\\\__________________________________________________________________
     ____/\\\________________\/\\\________/\\\\\\\\___/\\\____/\\\_____/\\\\\\\\___/\\/\\\\\\\______
     ___\/\\\________________\/\\\______/\\\/////\\\_\//\\\__/\\\____/\\\/////\\\_\/\\\/////\\\_____
     ___\//\\\_______________\/\\\_____/\\\\\\\\\\\___\//\\\/\\\____/\\\\\\\\\\\__\/\\\___\///______
     ____\///\\\_____________\/\\\____\//\\///////_____\//\\\\\____\//\\///////___\/\\\_____________
     ______\////\\\\\\\\\__/\\\\\\\\\__\//\\\\\\\\\\____\//\\\______\//\\\\\\\\\\_\/\\\_____________
     _________\/////////__\/////////____\//////////______\///________\//////////__\///______________*/



    public void initCompleate() {
        if (telemetry != null) {
            telemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
            telemetry.update();
        }
    }

    public void addTelemetry(String caption, Object value) { if (telemetry != null) telemetry.addData(caption, value); }

    public void clearTelemetry() { if (telemetry != null) telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetry != null) {
            if (telemetryAddLoopTime)
                telemetry.addData("Loop Time: ", getLoopFrequency());
            telemetry.update();
        }
    }

    public void debuggingTelemetry() {
        if (telemetry != null) {

            /*telemetry.addData("AMPS: ", scoring.getLiftAmperage());
            telemetry.addData("manualIntake: ", scoring.isManualIntakeEnabled());
            telemetry.addData("manualOuttake: ", scoring.isManualOuttakeEnabled());*/
            telemetry.addData("pixels in possession: ", scoring.getNumberOfStoredPixels());
            //telemetry.addData("last collected pixel: ", scoring.getLastCollectedPixel());

            telemetry.addData("hasGrabbedOnePixel", scoring.hasGrabbedOnePixel());
            telemetry.addData("hasGrabbedTwoPixels", scoring.hasGrabbedTwoPixels());

            telemetry.addData("swerve speed:", speed);
            telemetry.addData("usingDriveSensitivity", usingDriveSensitivity);
            telemetry.addData("PULLUP: ", override);

            //telemetry.addData("Sliders: ", scoring.getCurrentLiftPositionAverage());
            //telemetry.addData("Left: ", scoring.getLeftOuttakeEncoderPosition());
            //telemetry.addData("Right: ", scoring.getRightOuttakeEncoderPosition());

            telemetry.addData("lift MAX: ", scoring.getMAX());
            //telemetry.addData("lift MIN: ", scoring.getMIN());
            telemetry.addData("lift target: ", scoring.getTarget());

            //telemetry.addData("R: ", system.getR());
            //telemetry.addData("G: ", system.getG());
            //telemetry.addData("B: ", system.getB());

            //telemetry.addData("transfer sensor 1: ", system.getTransferDistance(Sensors.FOR_PIXEL_1, DistanceUnit.CM));
            //telemetry.addData("transfer sensor 2: ", system.getTransferDistance(Sensors.FOR_PIXEL_2, DistanceUnit.CM));

            telemetry.addData("is first pixel in transfer: ", scoring.isFirstPixelInTransfer());
            telemetry.addData("is second pixel in transfer: ", scoring.isSecondPixelInTransfer());
            //telemetry.addData("get actual number of pixels: ", system.getNumberOfPossessedPixels());*/

        }
    }
}
