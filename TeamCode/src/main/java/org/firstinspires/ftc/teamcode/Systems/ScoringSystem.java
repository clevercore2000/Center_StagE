package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.autoGetBackToIntermediary;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.autoGrabPixelsAfterCollectingTwoPixels;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.autoResetSystem;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.autoSpitAfterCollectingTwoPixels;
import static org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring.Outtake.safeRotationThreshold;
import static org.firstinspires.ftc.teamcode.WayFinder.Math.MathFormulas.sign;
import static org.firstinspires.ftc.teamcode.WayFinder.Math.MathFormulas.toPower;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring.Outtake;
import org.firstinspires.ftc.teamcode.Util.SensorEx.CompareRGB;

import java.util.ArrayList;
import java.util.List;

public class ScoringSystem implements Enums.Scoring, Enums {
    public static double numberOfInstanceCalls = 0;
    public static double numberOfAutonomusInstanceCalls = 0, numberOfTeleOpInstanceCalls = 0;
    public static OpMode opModeType = null;

    private Intake intake;
    private Outtake outtake;

    private double transferFirstSensorDistance, transferSecondSensorDistance;
    private double r, g, b; //deprecated. Not working to detect pixel color

    private RevColorSensorV3 transfer_pixel1, transfer_pixel2;

    private LinearOpMode opMode;

    /**Time values in milliseconds*/
    private final long timeForSpitting = 550;
    public final long timeForPixelsToBeScored = 3000;
    private final int timeToOpenGripper = 350;
    private final int timeToRotateSafelyWhenComingDown = 500;
    private final int timeToWaitForLiftBeforeRotating = 200;
    //TODO: determine optimal times


    /**Constants for colored pixels
     * WHITE
     * PURPLE
     * YELLOW
     * GREEN*/
    double rW = 2100, gW = 2206, bW = 2495;
    double rP = 1537, gP = 1565, bP = 2077;
    double rY = 1595, gY = 1508, bY = 1586;
    double rG = 1177, gG = 1406, bG = 1520;
    //TODO: find sensor values


    double rampColorThreshold = 50;
    //TODO: adjust threshold accordingly

    private CompareRGB whitePixelProcessor, purplePixelProcessor, yellowPixelProcessor, greenPixelProcessor;

    private static List<Pixels> pixelsInPossession = new ArrayList<>();
    private static List<Pixels> pixelRecognitions = new ArrayList<>();

    private static Pixels lastCollectedPixel = Pixels.YELLOW;
    private static StoredPixels numberOfCollectedPixels = StoredPixels.ONE;

    private static StoredPixels lastNumberOfCollectedPixels;

    private static boolean manualIntakeEnabled = true;
    private static boolean manualOuttakeEnabled = false;

    private static boolean lastManualIntakeEnabled;
    private static boolean lastManualOuttakeEnabled;

    private static boolean hasGrabbedOnePixel = false;
    private static boolean hasGrabbedTwoPixels = false;

    //sensor values (range between 0.25 - 8)
    private final double distanceCheckingIfPixelIsInTransfer = 1;
    private static boolean isFirstPixelInTransfer = true;
    private static boolean isSecondPixelInTransfer = false;

    private double manualLiftSensitivity = 100;
    private int manualLiftPower = 2;


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public ScoringSystem getInstance() { return this; }

    public ScoringSystem(LinearOpMode opMode, OpMode type)
    {
        this.opMode = opMode;
        opModeType = type;

        switch (opModeType) {
            case TELE_OP: {
                numberOfTeleOpInstanceCalls++;
            } break;
            case AUTONOMUS: {
                numberOfAutonomusInstanceCalls++;
            } break;
        }
        numberOfInstanceCalls++;

        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

        transfer_pixel1 = opMode.hardwareMap.get(RevColorSensorV3.class, "pixel1");
        transfer_pixel2 = opMode.hardwareMap.get(RevColorSensorV3.class, "pixel2");

        //initializeColorRecognition();
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    /**Setting positions for:
     *  -INTAKE:
     *      -arm
     *      -motor
     *  -OUTTAKE:
     *      -gripper
     *      -rotation
     *      -lift
     */
    public void setState(IntakeArmStates state) { intake.setState(state); }

    public void setState(IntakeMotorStates state) { intake.setState(state); }

    public void setState(OuttakeGripperStates state) { outtake.setState(state); }

    public void setState(OuttakeRotationStates state) { outtake.setState(state); }

    public void setState(LiftStates state) { outtake.setTarget(state);}

    public void setTarget(int target) { outtake.setTarget(target);}


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void initializeSystem() {
        if (opModeType == OpMode.TELE_OP) {
            resetSystem();

            getTo(Position.INTERMEDIARY);
            stopIntake();
        } else if (opModeType == OpMode.AUTONOMUS) {
            //tune this later
        }

        updateScoringSystem();
    }

    private void initializeColorRecognition() {
        whitePixelProcessor = new CompareRGB(rampColorThreshold, rW, gW, bW);
        purplePixelProcessor = new CompareRGB(rampColorThreshold, rP, gP, bP);
        yellowPixelProcessor = new CompareRGB(rampColorThreshold, rY, gY, bY);
        greenPixelProcessor = new CompareRGB(rampColorThreshold, rG, gG, bG);
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private Pixels findFirstPixel() {
        //if (pixelRecognitions.size() == 0)
            return Pixels.UNIDENTIFIED;

        //return pixelRecognitions.get(0);
    }

    private Pixels findSecondPixel() {
        //if (pixelRecognitions.size() == 0)
            return Pixels.UNIDENTIFIED;

        /*if (pixelRecognitions.size() == 1) {
            if (pixelsInPossession.get(0) == Pixels.UNIDENTIFIED) { return pixelsInPossession.get(0); }
            else { return Pixels.UNIDENTIFIED; }
        }

        Pixels firstRecognition = pixelRecognitions.get(0);

        for(int i = 1; i < pixelRecognitions.size(); i++) {
            if (pixelRecognitions.get(i) != firstRecognition) {
                return pixelRecognitions.get(i);
            }
        }

        return firstRecognition;*/
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private Pixels recognizeColor() {
        if (whitePixelProcessor.isSameColor(r, g, b)) return Pixels.WHITE;
        if (purplePixelProcessor.isSameColor(r, g, b)) return Pixels.PURPLE;
        if (yellowPixelProcessor.isSameColor(r, g, b)) return Pixels.YELLOW;
        if (greenPixelProcessor.isSameColor(r, g, b)) return Pixels.GREEN;

        return Pixels.NONE;
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private void updateScoringSystem() {
        //intake.update();
        outtake.update();
    }

    private void updateAutoActions() {
        updateSensors();
        updateCollectedPixels();
        updateCorrection();
        updateState();
    }

    public void update() {
        updateAutoActions();
        updateScoringSystem();
    }

    protected void updateCollectedPixels(){
        if (pixelsInPossession.size() == 0)
            numberOfCollectedPixels = StoredPixels.ZERO;
        else if (pixelsInPossession.size() == 1)
            numberOfCollectedPixels = StoredPixels.ONE;
        else if (pixelsInPossession.size() == 2)
            numberOfCollectedPixels = StoredPixels.TWO;
    }

    protected void updateSensors() {
        isFirstPixelInTransfer = transferFirstSensorDistance < distanceCheckingIfPixelIsInTransfer;
        isSecondPixelInTransfer = transferSecondSensorDistance < distanceCheckingIfPixelIsInTransfer;

        //deprecated. Just slowing the code at this point
        /* recognition = recognizeColor();
        if (recognition != Pixels.NONE)
            pixelRecognitions.add(recognition);*/

        if (isFirstPixelInTransfer && pixelsInPossession.size() == 0) { pixelsInPossession.add(findFirstPixel()); }
        else if (isFirstPixelInTransfer && pixelsInPossession.size() == 1 && hasGrabbedOnePixel) { pixelsInPossession.add(findSecondPixel()); }
        else if (isSecondPixelInTransfer && pixelsInPossession.size() == 1) { pixelsInPossession.add(findSecondPixel()); }

        lastCollectedPixel = (pixelsInPossession.size() == 0) ? lastCollectedPixel : pixelsInPossession.get(pixelsInPossession.size() - 1);
    }

    protected void updateCorrection() {
        if (numberOfCollectedPixels == StoredPixels.TWO && isIsFirstPixelInTransfer() && isIsSecondPixelInTransfer()) {
            hasGrabbedOnePixel = false;
            hasGrabbedTwoPixels = false;
        }

        if (hasGrabbedOnePixel) {
            if (isIsSecondPixelInTransfer()) { hasGrabbedOnePixel = false; }
        }

        if (hasGrabbedTwoPixels) {
             if (numberOfCollectedPixels == StoredPixels.ONE && isIsFirstPixelInTransfer()) { hasGrabbedOnePixel = false; }
        }
    }

    protected void updateState() {
        if (intake.isIntakeConstrained())
            spitForTimeThen(300, AfterSpitting.STOP);
        
        switch (numberOfCollectedPixels) {
            case ZERO: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(false);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) {
                    getTo(Position.INTERMEDIARY);
                    outtake.setMaxManualTarget(LiftStates.INTERMEDIARY);
                }

                if (isIntakeConstrained()) { spitForTimeThen(timeForSpitting, AfterSpitting.GET_BACK); }
            } break;
            case ONE: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(true);

                if(hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) {
                    outtake.setMaxManualTarget(LiftStates.HIGH);
                }

                if (isIntakeConstrained()) { spitForTimeThen(timeForSpitting, AfterSpitting.GET_BACK); }
            } break;
            case TWO: {
                setManualIntakeEnabled(false);
                setManualOuttakeEnabled(true);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged())  {
                    if (!hasGrabbedOnePixel) {
                        setManualOuttakeEnabled(false);

                        getTo(Position.INTERMEDIARY);

                        if (autoSpitAfterCollectingTwoPixels) { spitForTimeThen(timeForSpitting, AfterSpitting.STOP); }

                        if (autoGrabPixelsAfterCollectingTwoPixels) { autoCollectPixels(); }
                    }

                    setManualOuttakeEnabled(true);
                    outtake.setMaxManualTarget(LiftStates.HIGH);

                    stopIntake();
                }
            } break;
        }

        lastNumberOfCollectedPixels = numberOfCollectedPixels;
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void resetLift() { outtake.resetLift(); }

    public void resetLiftEncoders() { outtake.resetEncoders();}

    public void resetSystem() {
        if (autoResetSystem) {
            setTarget(safeRotationThreshold);
            setState(OuttakeRotationStates.COLLECT);
            sleep(700, Update.OUTTAKE);

            while (!opMode.isStarted() && !opMode.isStopRequested() && !isLiftConstrained()) { resetLift(); }

            resetLiftEncoders();
        }
    }

    public void resetNumberOfCollectedPixelsToZero() {
        lastCollectedPixel = Pixels.NONE;
        numberOfCollectedPixels = StoredPixels.ZERO;

        removePixelsUntilRemain(0);
        removeRecognitions();
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private void sleep(long milliseconds, Update type) {
        ElapsedTime temp = new ElapsedTime();
        temp.startTime();

        switch (type) {
            case INTAKE: while (temp.milliseconds() <= milliseconds) { //intake.update();
                intake.read(); } break;
            case OUTTAKE: while (temp.milliseconds() <= milliseconds) { outtake.update(); outtake.read(); } break;
            case ALL: while (temp.milliseconds() <= milliseconds) { update(); read(); } break;
            case SCORING_SYSTEM: while (temp.milliseconds() <= milliseconds) { updateScoringSystem(); intake.read(); outtake.read(); } break;
            case SENSORS: while (temp.milliseconds() <= milliseconds) { updateSensors(); read(); } break;
            case NONE: while (temp.milliseconds() <= milliseconds) { read(); } break;
        }

    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    //Should be use with caution... too powerful
    public void removePixelsUntilRemain(double remaining) {
        while (pixelsInPossession.size() > remaining) { pixelsInPossession.remove(pixelsInPossession.size() - 1); }

        if (remaining == 0) { removeRecognitions(); }
    }

    public void removeRecognitions() { pixelRecognitions = new ArrayList<>(); }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    /**Debugging*/
    public  double getLeftOuttakeEncoderPosition() { return outtake.getLeftPosition(); }

    public  double getRightOuttakeEncoderPosition() { return outtake.getRightPosition(); }

    public OpMode getOpModeType() { return opModeType; }

    public static double getNumberOfInstanceCalls() { return numberOfInstanceCalls; }

    public double getProfileVelocity() { return outtake.getProfileVelocity(); }

    public double getLiftAmperage() { return outtake.getLiftAmperage(); }

    public double getIntakeAmperage() { return intake.getIntakeAmperage(); }

    public Pixels getLastCollectedPixel() { return lastCollectedPixel; }

    public StoredPixels getNumberOfStoredPixels() { return numberOfCollectedPixels; }

    public StoredPixels getLastNumberOfStoredPixels() { return lastNumberOfCollectedPixels; }

    public double getNumberOfPossessedPixels() { return pixelsInPossession.size(); }

    public int getCurrentLiftPositionAverage() { return outtake.getCurrentPositionAverage(); }

    public double getR() { return r; }

    public double getG() { return g; }

    public double getB() { return b; }

    public double getMAX() { return outtake.getMAX(); }

    public double getMIN() { return outtake.getMIN(); }

    public double getTarget() { return outtake.getTarget(); }

    public double getTransferDistance(Sensors sensor, DistanceUnit distanceUnit) {
        switch (sensor) {
            case FOR_PIXEL_1: { return transfer_pixel1.getDistance(distanceUnit); }
            case FOR_PIXEL_2: { return transfer_pixel2.getDistance(distanceUnit); }
            default: { return 0; }
        }
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    /**User's responsibility to call this in the OpMode's logic*/
    public void setHasGrabbedOnePixel(boolean hasGrabbed) { hasGrabbedOnePixel = hasGrabbed; }

    public void setHasGrabbedTwoPixels(boolean hasGrabbed) { hasGrabbedTwoPixels = hasGrabbed; }

    protected void setManualIntakeEnabled(boolean isEnabled) {
        lastManualIntakeEnabled = manualIntakeEnabled;
        manualIntakeEnabled = isEnabled;
    }

    protected void setManualOuttakeEnabled(boolean isEnabled) {
        lastManualOuttakeEnabled = manualOuttakeEnabled;
        manualOuttakeEnabled = isEnabled;
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public boolean isManualIntakeEnabled() { return manualIntakeEnabled; }

    public boolean isManualOuttakeEnabled() { return  manualOuttakeEnabled;}

    public boolean isIntakeConstrained() { return intake.isIntakeConstrained(); }

    public boolean isLiftConstrained() { return outtake.isLiftConstrained();
    }

    public boolean isIsFirstPixelInTransfer() { return isFirstPixelInTransfer; }

    public boolean isIsSecondPixelInTransfer() { return isSecondPixelInTransfer; }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    protected boolean hasManualIntakeEnableJustChanged() { return lastManualIntakeEnabled != manualIntakeEnabled; }

    protected boolean hasManualOuttakeEnableJustChanged() { return lastManualOuttakeEnabled != manualOuttakeEnabled; }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    /**Multi-action methods*/

    public void score(Score scoringOption) {
        boolean hasToWaitForLift = (outtake.getLiftState() == LiftStates.COLLECT);

        switch (scoringOption) {
            case LOW: {setState(LiftStates.LOW); } break;
            case MID: { setState(LiftStates.MID); } break;
            case HIGH: {setState(LiftStates.HIGH); } break;
            default: {}
        }

        setState(OuttakeGripperStates.OPEN);

        if (hasToWaitForLift) { sleep(timeToWaitForLiftBeforeRotating, Update.OUTTAKE); }

        setState(OuttakeRotationStates.SCORE);
    }

    public void getTo(Position desiredPosition) {
        if (outtake.getLiftState() != LiftStates.COLLECT || outtake.getLiftState() != LiftStates.INTERMEDIARY) {
            if (outtake.getTarget() < safeRotationThreshold && outtake.getLiftState() != LiftStates.COLLECT && outtake.getLiftState() != LiftStates.INTERMEDIARY) {
                outtake.setTarget(safeRotationThreshold + 30);
                sleep(200, Update.OUTTAKE);
            }

            setState(OuttakeRotationStates.COLLECT);
            sleep(timeToRotateSafelyWhenComingDown, Update.OUTTAKE);
        }

        switch (desiredPosition) {
            case COLLECT: {
                if (!hasGrabbedOnePixel && !hasGrabbedTwoPixels) {
                    setState(OuttakeGripperStates.CLOSED);
                    sleep(timeToOpenGripper, Update.OUTTAKE);
                }

                setState(LiftStates.COLLECT);
            } break;

            case INTERMEDIARY: {
                if (!hasGrabbedOnePixel && !hasGrabbedTwoPixels) { setState(OuttakeGripperStates.CLOSED); }
                    else { setState(OuttakeGripperStates.OPEN); }

                setState(LiftStates.INTERMEDIARY);
            }

        }

    }

    public void pixels(PixelActions action) {
        switch (action) {
            case GRAB: {
                setState(OuttakeGripperStates.OPEN);
                sleep(timeToOpenGripper, Update.OUTTAKE);

                setState(LiftStates.INTERMEDIARY);
                sleep(200, Update.OUTTAKE);

            } break;

            case SCORE: {
                setState(OuttakeGripperStates.CLOSED);
                sleep(timeForPixelsToBeScored, Update.OUTTAKE);

                setState(OuttakeRotationStates.COLLECT);

                if (autoGetBackToIntermediary) {
                    sleep(timeToWaitForLiftBeforeRotating, Update.OUTTAKE);
                    getTo(Position.INTERMEDIARY);
                }

                removePixelsUntilRemain((hasGrabbedOnePixel && numberOfCollectedPixels == StoredPixels.TWO) ? 1 : 0);

                setHasGrabbedOnePixel(false);
                setHasGrabbedTwoPixels(false);

            } break;

            case COLLECT_GRAB: {
                if (getNumberOfStoredPixels() == StoredPixels.ONE) { setHasGrabbedOnePixel(true); }
                else { setHasGrabbedOnePixel(false); }

                if (getNumberOfStoredPixels() == StoredPixels.TWO) { setHasGrabbedTwoPixels(true);  }
                else { setHasGrabbedTwoPixels(false); }

                getTo(Position.COLLECT);
                pixels(PixelActions.GRAB);
            }
        }
    }

    private void autoCollectPixels() {
        getTo(Position.COLLECT);
        sleep(100, Update.OUTTAKE);

        setState(OuttakeGripperStates.OPEN);
        sleep(timeToOpenGripper,Update.OUTTAKE);

        hasGrabbedTwoPixels = true;
    }

    public void manualControlLift(double value) {
        if (Math.abs(value) > 0.1) {
            int target = (int) (getCurrentLiftPositionAverage() + sign(value) * toPower(value, manualLiftPower) * manualLiftSensitivity);
            setTarget(target);
        }
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void startIntake(IntakeArmStates desiredArmState) {
        setState(desiredArmState);
        setState(IntakeMotorStates.COLLECT);
    }

    public void stopIntake() {
        setState(IntakeArmStates.UP);
        setState(IntakeMotorStates.STOP);
    }

    public void spitIntake() {
        setState(IntakeArmStates.DOWN);
        setState(IntakeMotorStates.SPIT);
    }

    public void spitForTimeThen(long time, AfterSpitting action) {
        IntakeArmStates armState = intake.getArmState();

        spitIntake();
        sleep(time,Update.INTAKE);

        if (action == AfterSpitting.STOP) { stopIntake(); }
            else if (action == AfterSpitting.GET_BACK) { startIntake(armState); }
        //intake.update();
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private void readSensors() {
        transferFirstSensorDistance = transfer_pixel1.getDistance(DistanceUnit.CM);
        transferSecondSensorDistance = transfer_pixel2.getDistance(DistanceUnit.CM);
    }

    public void read() {
        intake.read();
        outtake.read();
        readSensors();
    }

}
