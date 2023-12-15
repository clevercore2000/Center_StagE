package org.firstinspires.ftc.teamcode.Systems.Robot;

import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.usingOpenCvCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.Systems.OtherSystem;
import org.firstinspires.ftc.teamcode.Systems.ScoringSystem;
import org.firstinspires.ftc.teamcode.Util.SensorEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

import javax.annotation.Nullable;

public class CleverBot {
    public ScoringSystem scoring;
    public OtherSystem other;
    private CleverSwerve swerve;

    private HubBulkRead core;
    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;
    private Telemetry telemetry;

    //default constrains
    private CleverData constrains = new CleverData()
            .add(Enums.Swerve.Localizers.ROADRUNNER)
            .add(Enums.Swerve.MotionPackage.CUSTOM)
            .add(Enums.OpMode.TELE_OP)
            .setLockedWheelStyle(Enums.Swerve.LockedWheelPositions.DIAMOND)
            .allowOtherUsageBeforeEndgame(false)
            .setUsingAprilTag(false)
            .setUsingOpenCv(false)
            .;



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

    public CleverBot construct(LinearOpMode opMode) {
        this.opMode = opMode;

        scoring = new ScoringSystem(opMode, constrains.opModeType);
        other = new OtherSystem(opMode);
        swerve = new CleverSwerve(opMode, constrains.localizer, constrains.opModeType);

        openCvCamera = (usingOpenCvCamera) ? new Camera(opMode, telemetry) : null;
        aprilTagCamera = (usingAprilTagCamera) ? new AprilTagCamera(opMode, telemetry) : null;

        core = new HubBulkRead(opMode.hardwareMap);

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
        scoring.read();
        other.read();
        swerve.read();
    }

    public void updateLift() { scoring.update();}

    public void updatePullUp() { other.update(); }

    public void updateSwerve() { swerve.update(); }

    public void updateAll() {
        scoring.update();
        other.update();
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

    public void setRobotPosition(Pose newPose) { swerve.setPoseEstimate(newPose); }

    public void startEndgameTimer() { other.start(); }

    public void initializeScoring() { scoring.initializeSystem(); }

    public void drive(double x, double y, double heading) { swerve.drive(x, y, heading); }

    public void getStartingLoopTime() { startLoopTime = endLoopTime; }

    public void getEndingLoopTime() { endLoopTime = System.nanoTime(); }

    public double getLoopFrequency() { return secondsToNanoseconds / (endLoopTime - startLoopTime); }

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
            /*telemetry.addData("AMPS: ", system.getLiftAmperage());
        telemetry.addData("manualIntake: ", system.isManualIntakeEnabled());
        telemetry.addData("manualOuttake: ", system.isManualOuttakeEnabled());
        telemetry.addData("pixels in possession: ", system.getNumberOfStoredPixels());
        telemetry.addData("last collected pixel: ", system.getLastCollectedPixel());

        //telemetry.addData("lift MAX: ", system.getMAX());
        //telemetry.addData("lift MIN: ", system.getMIN());
        telemetry.addData("lift target: ", system.getTarget());

        //telemetry.addData("R: ", system.getR());
        //telemetry.addData("G: ", system.getG());
        //telemetry.addData("B: ", system.getB());

        telemetry.addData("transfer sensor 1: ", system.getTransferDistance(Sensors.FOR_PIXEL_1, DistanceUnit.CM));
        telemetry.addData("transfer sensor 2: ", system.getTransferDistance(Sensors.FOR_PIXEL_2, DistanceUnit.CM));

        telemetry.addData("is first pixel in transfer: ", system.isIsFirstPixelInTransfer());
        telemetry.addData("is second pixel in transfer: ", system.isIsSecondPixelInTransfer());
        telemetry.addData("get actual number of pixels: ", system.getNumberOfPossessedPixels());*/
        }
    }
}
