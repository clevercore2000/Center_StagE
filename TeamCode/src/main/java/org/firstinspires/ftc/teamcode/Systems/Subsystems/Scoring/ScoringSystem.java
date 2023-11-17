package org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring;

import static org.firstinspires.ftc.teamcode.Util.Math.MathFormulas.sign;
import static org.firstinspires.ftc.teamcode.Util.Math.MathFormulas.toPower;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Systems.Enums;
import org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.SensorEx.CompareRGB;
import org.firstinspires.ftc.teamcode.Util.SensorEx.HubBulkRead;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class ScoringSystem {
    public static ScoringSystem instance = null;

    private static Intake intake;
    private static Outtake outtake;

    private static RevColorSensorV3 transfer_pixel1, transfer_pixel2;
    private static RevColorSensorV3 ramp_sensor;

    private static LinearOpMode opMode;

    private static ElapsedTime timerForSpitting;

    /**Time values in milliseconds*/
    private final double timeForSpitting = 300;
    public static double timeForPixelsToBeScored = 400;
    private final int timeToOpenGripper = 350;
    private final int timeBeforeRotate = 350;
    private final int timeToRotateWithoutBreaking = 200;
    //TODO: determine optimal times


    /**Constants for colored pixels
     * WHITE
     * PURPLE
     * YELLOW
     * GREEN*/
    double rW = 82, gW = 147, bW = 130;
    double rP = 40, gP = 60, bP = 82;
    double rY = 52, gY = 76, bY = 90;
    double rG = 15, gG = 51, bG = 90;
    //TODO: find sensor values

    private double r, g, b;


    double rampColorThreshold = 20;
    //TODO: adjust threshold accordingly

    private static CompareRGB whitePixelProcessor, purplePixelProcessor, yellowPixelProcessor, greenPixelProcessor;

    private static List<Enums.Pixels> pixelsInPossession = new ArrayList<>();
    private static List<Enums.Pixels> pixelRecognitions = new ArrayList<>();

    private static Enums.Pixels lastCollectedPixel = Enums.Pixels.YELLOW;
    private static Enums.StoredPixels numberOfCollectedPixels = Enums.StoredPixels.ONE;

    private static Enums.StoredPixels lastNumberOfCollectedPixels;

    private static boolean manualIntakeEnabled = true;
    private static boolean manualOuttakeEnabled = false;

    private static boolean lastManualIntakeEnabled;
    private static boolean lastManualOuttakeEnabled;

    private static boolean hasGrabbedOnePixelAlready = false;

    private final double distanceCheckingIfPixelIsInTransfer = 20;

    private double manualLiftSensitivity = 100;
    private int manualLiftPower = 2;


    public static ScoringSystem getInstance(LinearOpMode opModE) {
        if (instance == null)
            instance = new ScoringSystem(opModE);

        opMode = opModE;
        return instance;
    }

    public ScoringSystem(LinearOpMode opMode)
    {
        this.opMode = opMode;
        pixelsInPossession.add(lastCollectedPixel);

        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

        transfer_pixel1 = opMode.hardwareMap.get(RevColorSensorV3.class, "pixel1");
        transfer_pixel2 = opMode.hardwareMap.get(RevColorSensorV3.class, "pixel2");
        ramp_sensor = opMode.hardwareMap.get(RevColorSensorV3.class, "ranp_sensor");


        timerForSpitting = new ElapsedTime();
        timerForSpitting.startTime();

        initializeColorRecognition();

        autoReset();
        initializeSystem();

    }


    /**Setting positions for:
     *  -INTAKE:
     *      -arm
     *      -motor
     *  -OUTTAKE:
     *      -gripper
     *      -rotation
     *      -lift
     */
    public void setState(Enums.IntakeArmStates state) { intake.setState(state); }

    public void setState(Enums.IntakeMotorStates state) { intake.setState(state); }

    public void setState(Enums.OuttakeGripperStates state) { outtake.setState(state); }

    public void setState(Enums.OuttakeRotationStates state) { outtake.setState(state); }

    public void setState(Enums.LiftStates state) { outtake.setTarget(state);}

    public void setTarget(int target) { outtake.setTarget(target); }




    /**Call this when you init the robot without competition procedures*/
    public void resetNumberOfCollectedPixelsToZero() {
        lastCollectedPixel = Enums.Pixels.NONE;
        numberOfCollectedPixels = Enums.StoredPixels.ZERO;

        removePixelsUntilRemain(0);
        eraseRecognitions();
    }

    public void initializeSystem() {
        getTo(Enums.Position.INTERMEDIARY);
        stopIntake();
    }

    private void initializeColorRecognition() {
        whitePixelProcessor = new CompareRGB(rampColorThreshold, rW, gW, bW);
        purplePixelProcessor = new CompareRGB(rampColorThreshold, rP, gP, bP);
        yellowPixelProcessor = new CompareRGB(rampColorThreshold, rY, gY, bY);
        greenPixelProcessor = new CompareRGB(rampColorThreshold, rG, gG, bG);
    }
    
    protected void setManualIntakeEnabled(boolean isEnabled) {
        lastManualIntakeEnabled = manualIntakeEnabled;
        manualIntakeEnabled = isEnabled;
    }

    protected void setManualOuttakeEnabled(boolean isEnabled) {
        lastManualOuttakeEnabled = manualOuttakeEnabled;
        manualOuttakeEnabled = isEnabled;
    }




    private Enums.Pixels recognizeColor() {
        r = ramp_sensor.red() / 100;
        g = ramp_sensor.green() / 100;
        b = ramp_sensor.blue() / 100;

        if (whitePixelProcessor.isSameColor(r, g, b)) return Enums.Pixels.WHITE;
        if (purplePixelProcessor.isSameColor(r, g, b)) return Enums.Pixels.PURPLE;
        if (yellowPixelProcessor.isSameColor(r, g, b)) return Enums.Pixels.YELLOW;
        if (greenPixelProcessor.isSameColor(r, g, b)) return Enums.Pixels.GREEN;

        return Enums.Pixels.NONE;

    }

    protected void updateCollectedPixels(){
        if (pixelsInPossession.size() == 0)
            numberOfCollectedPixels = Enums.StoredPixels.ZERO;
        else if (pixelsInPossession.size() == 1)
            numberOfCollectedPixels = Enums.StoredPixels.ONE;
        else if (pixelsInPossession.size() == 2)
            numberOfCollectedPixels = Enums.StoredPixels.TWO;
        else if (pixelsInPossession.size() >= 3) numberOfCollectedPixels = Enums.StoredPixels.TOO_MANY;
    }

    protected void updateSensors() {
        boolean isFirstPixelInTransfer = transfer_pixel1.getDistance(DistanceUnit.MM) < distanceCheckingIfPixelIsInTransfer;
        boolean isSecondPixelInTransfer = transfer_pixel2.getDistance(DistanceUnit.MM) < distanceCheckingIfPixelIsInTransfer;

        Enums.Pixels recognition = recognizeColor();
        if (recognition != Enums.Pixels.NONE)
            pixelRecognitions.add(recognition);

        if (isFirstPixelInTransfer && pixelsInPossession.size() == 0) { pixelsInPossession.add(findFirstPixel()); }
            else if (isSecondPixelInTransfer && pixelsInPossession.size() > 0) { pixelsInPossession.add(findSecondPixel()); }

        lastCollectedPixel = (pixelsInPossession.size() == 0) ? lastCollectedPixel : pixelsInPossession.get(pixelRecognitions.size() - 1);

        if (timerForSpitting.milliseconds() < timeForSpitting) { updateCollectedPixels(); }


        if (isIntakeConstrained()) { numberOfCollectedPixels = Enums.StoredPixels.TOO_MANY; }


        switch (numberOfCollectedPixels) {
            case ZERO: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(false);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) { getTo(Enums.Position.INTERMEDIARY); }
            } break;
            case ONE: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(true);
            } break;
            case TWO: {
                setManualIntakeEnabled(false);
                stopIntake();

                if ((hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()))  {
                    if (!hasGrabbedOnePixelAlready) {
                        if (lastNumberOfCollectedPixels != Enums.StoredPixels.TOO_MANY) { numberOfCollectedPixels = Enums.StoredPixels.TOO_MANY; }
                            else { autoCollectPixels(); }
                    } else { setManualOuttakeEnabled(true); }
                }

            } break;
            case TOO_MANY: {
                setManualIntakeEnabled(false);
                setManualOuttakeEnabled(false);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) {
                    timerForSpitting.reset();
                    getTo(Enums.Position.INTERMEDIARY);
                }

                if (timerForSpitting.milliseconds() < timeForSpitting) {
                    setState(Enums.IntakeMotorStates.SPIT);
                    setState(Enums.IntakeArmStates.UP);
                }
            } break;
        }

        lastNumberOfCollectedPixels = numberOfCollectedPixels;
    }

    private Enums.Pixels findFirstPixel() {
        if (pixelRecognitions.size() == 0)
            return Enums.Pixels.UNIDENTIFIED;

        return pixelRecognitions.get(0);
    }

    private Enums.Pixels findSecondPixel() {
        if (pixelRecognitions.size() == 0)
            return Enums.Pixels.UNIDENTIFIED;

        if (pixelRecognitions.size() == 1) {
            if (pixelsInPossession.get(0) == Enums.Pixels.UNIDENTIFIED) { return pixelsInPossession.get(0); }
            else { return Enums.Pixels.UNIDENTIFIED; }
        }

        Enums.Pixels firstRecognition = pixelRecognitions.get(0);

        for(int i = 1; i < pixelRecognitions.size(); i++) {
            if (pixelRecognitions.get(i) != firstRecognition) {
                return pixelRecognitions.get(i);
            }
        }

        return firstRecognition;
    }

    private void autoCollectPixels() {
        setManualOuttakeEnabled(true);

        getTo(Enums.Position.COLLECT);
        pixels(Enums.PixelActions.GRAB);
    }

    public void autoReset() {
        setState(Enums.LiftStates.LOW);

        ElapsedTime temp = new ElapsedTime();
        temp.startTime();

        while (temp.milliseconds() <= 400) { update(); }
        while(!opMode.isStarted() && !opMode.isStopRequested() && !isLiftConstrained()) { resetLift(); }

        resetLiftEncoders();
    }

    public void manualControlLift(double value) {
        if (Math.abs(value) > 0.1) {
            int target = (int) (getCurrentLiftPositionAverage() + sign(value) * toPower(value, manualLiftPower) * manualLiftSensitivity);
            setTarget(target);
        }
    }

    private void updateScoringSystem() {
        intake.update();
        outtake.update();
    }

    public void update() {
        updateSensors();
        updateScoringSystem();
    }


    public void resetLift() { outtake.resetLift(); }

    public void resetLiftEncoders() { outtake.resetEncoders(); }


    /**Should be use with caution... too powerful*/
    public void removePixelsUntilRemain(double remaining) {
        while (pixelsInPossession.size() > remaining) { pixelsInPossession.remove(pixelsInPossession.size() - 1); }
    }

    public void eraseRecognitions() { pixelRecognitions = new ArrayList<>(); }


    /**Debugging*/
    public double getProfileVelocity() { return outtake.getProfileVelocity(); }

    public double getLiftAmperage() { return outtake.getLiftAmperage(); }

    public Enums.Pixels getLastCollectedPixel() { return lastCollectedPixel; }

    public Enums.StoredPixels getNumberOfStoredPixels() { return numberOfCollectedPixels; }

    public int getCurrentLiftPositionAverage() { return outtake.getCurrentPositionAverage(); }

    public double getR() { return  r; }

    public double getG() { return g; }

    public double getB() { return b; }

    public double getTime(Enums.Timers timer, TimeUnit unit) {
        switch (timer) {
            case TIMER_FOR_SPITTING: { return timerForSpitting.time(unit); }
            default: { return 0; }
        }
    }



    /**User's responsibility to call this in the OpMode's logic*/
    public void setHasGrabbedOnePixelAlready(boolean hasGrabbed) { hasGrabbedOnePixelAlready = hasGrabbed; }

    public boolean isManualIntakeEnabled() { return manualIntakeEnabled; }

    public boolean isManualOuttakeEnabled() { return  manualOuttakeEnabled; }

    public boolean isIntakeConstrained() { return intake.isIntakeConstrained(); }

    public boolean isLiftConstrained() { return outtake.isLiftConstrained(); }

    protected boolean hasManualIntakeEnableJustChanged() { return lastManualIntakeEnabled != manualIntakeEnabled; }

    protected boolean hasManualOuttakeEnableJustChanged() { return lastManualOuttakeEnabled != manualOuttakeEnabled; }





    /**Multi-action methods*/

    public void score(Enums.Scoring scoringOption) {

        switch (scoringOption) {
            case LOW: {setState(Enums.LiftStates.LOW); } break;
            case MID: { setState(Enums.LiftStates.MID); } break;
            case HIGH: {setState(Enums.LiftStates.HIGH); } break;
            default: {}
        }


        setState(Enums.OuttakeRotationStates.SCORE);
        setState(Enums.OuttakeGripperStates.OPEN);
    }

    public void getTo(Enums.Position desiredPosition) {
        setState(Enums.OuttakeRotationStates.COLLECT);

        try { Thread.sleep(timeToOpenGripper); }
        catch (InterruptedException e) { throw new RuntimeException(e); }

        switch (desiredPosition) {
            case COLLECT: {
                setState(Enums.OuttakeGripperStates.CLOSED);

                outtake.update();

                try { Thread.sleep(timeToOpenGripper); }
                    catch (InterruptedException e) { throw new RuntimeException(e); }

                setState(Enums.LiftStates.COLLECT);
            } break;

            case INTERMEDIARY: {
                setState(Enums.OuttakeGripperStates.OPEN);

                if (outtake.getLiftState() != Enums.LiftStates.COLLECT) {
                    outtake.update();

                    try { Thread.sleep(timeBeforeRotate); }
                    catch (InterruptedException e) { throw new RuntimeException(e); }
                }

                setState(Enums.LiftStates.INTERMEDIARY);
            }

        }

    }

    public void pixels(Enums.PixelActions action) {
        switch (action) {
            case GRAB: {
                setState(Enums.OuttakeGripperStates.OPEN);
                outtake.update();

                try { Thread.sleep(timeToOpenGripper); }
                catch (InterruptedException e) { throw new RuntimeException(e); }

                setState(Enums.LiftStates.INTERMEDIARY);

            } break;

            case SCORE: {
                setState(Enums.OuttakeGripperStates.CLOSED);
                outtake.update();

                try { Thread.sleep((long) timeForPixelsToBeScored); }
                catch (InterruptedException e) { throw new RuntimeException(e); }

                setState(Enums.OuttakeRotationStates.COLLECT);
                outtake.setLiftAscending(false);

                try { Thread.sleep(timeToRotateWithoutBreaking); }
                catch (InterruptedException e) { throw new RuntimeException(e); }

                outtake.setLiftAscending(true);
                getTo(Enums.Position.INTERMEDIARY);
                removePixelsUntilRemain(hasGrabbedOnePixelAlready ? 1 : 0);

            } break;
        }
    }

    public void startIntake(Enums.IntakeArmStates desiredArmState) {
        setState(desiredArmState);
        setState(Enums.IntakeMotorStates.COLLECT);
    }

    public void stopIntake() {
        setState(Enums.IntakeArmStates.UP);
        setState(Enums.IntakeMotorStates.STOP);
    }

}
