package org.firstinspires.ftc.teamcode.hardware.Robot.Systems;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.autoGetBackToIntermediary;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.autoGrabPixelsAfterCollectingTwoPixels;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.autoResetSystem;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.autoSpitAfterCollectingTwoPixels;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.multithreading;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.transfer_color_sensor_down;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.transfer_color_sensor_up;
import static org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Outtake.outOfTransferThreshold;
import static org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Outtake.safeRotationThreshold;
import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.MathFormulas.toPower;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring.Outtake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ScoringSystem implements Enums.Scoring, Enums {
    private CleverBot robotInstance;
    public static boolean justScoredPixels;

    private ElapsedTime ifBugsOccur;
    private long timeToWaitForSlidersCorrection = 1300; //ms

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
    public final long timeForPixelsToBeScored = 1000;
    private final int timeToOpenGripper = 385;
    private final int timeToRotateSafelyWhenComingDown = 950;
    private final int timeToWaitForLiftBeforeRotating = 300;


    private static List<Pixels> pixelsInPossession = Arrays.asList(Pixels.YELLOW);

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

    public void setRobotInstance(CleverBot robotInstance) {
        this.robotInstance = robotInstance;
        outtake.setRobotInstance(robotInstance);
    }

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

        ifBugsOccur = new ElapsedTime();
        ifBugsOccur.startTime();

        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

        transfer_pixel1 = opMode.hardwareMap.get(RevColorSensorV3.class, transfer_color_sensor_down);
        transfer_pixel2 = opMode.hardwareMap.get(RevColorSensorV3.class, transfer_color_sensor_up);
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

    public void setTarget(int target, boolean override, boolean overrideRotation) { outtake.setTarget(target, override, overrideRotation);}


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void initializeSystem() {
        if (opModeType == OpMode.TELE_OP) {
            if (autoResetSystem)
                resetSystem();

            getTo(Position.COLLECT);
            stopIntake();
        } else if (opModeType == OpMode.AUTONOMUS) {
            stopIntake();
        }

    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void updateLift() { outtake.update(); }

    public void updateAutoActions() {
        updateSensors();
        updateCollectedPixels();
        if (outtake.getCurrentPositionAverage() > outOfTransferThreshold || outtake.getRotationState() == OuttakeRotationStates.SCORE)
            updateCorrection();
        updateState();
    }

    public void update() {
        updateAutoActions();
        updateLift();
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

        boolean outtakeOutOfTransfer = outtake.getCurrentPositionAverage() > outOfTransferThreshold || outtake.getRotationState() == OuttakeRotationStates.SCORE;

        if (isFirstPixelInTransfer && pixelsInPossession.size() == 0) { pixelsInPossession.add(Pixels.UNIDENTIFIED); }
        else if (isFirstPixelInTransfer && pixelsInPossession.size() == 1 && hasGrabbedOnePixel && outtakeOutOfTransfer) { pixelsInPossession.add(Pixels.UNIDENTIFIED); }
        else if (isSecondPixelInTransfer && pixelsInPossession.size() == 1) { pixelsInPossession.add(Pixels.UNIDENTIFIED); }
    }

    protected void updateCorrection() {
        if (numberOfCollectedPixels == StoredPixels.TWO && isFirstPixelInTransfer && isSecondPixelInTransfer) {
            hasGrabbedOnePixel = false;
            hasGrabbedTwoPixels = false;
        }

        if (hasGrabbedOnePixel) {
            if (isSecondPixelInTransfer) { hasGrabbedOnePixel = false; }
            if (!isFirstPixelInTransfer) { removePixelsUntilRemain(1); }
        }

        if (hasGrabbedTwoPixels) {
             if (numberOfCollectedPixels == StoredPixels.ONE && isFirstPixelInTransfer) { hasGrabbedOnePixel = false; }
        }
    }

    protected void updateState() {
        if (intake.isIntakeConstrained())
            spitForTimeThen(300, AfterSpitting.GET_BACK);

        switch (numberOfCollectedPixels) {
            case ZERO: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(false);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) {
                    outtake.setMaxManualTarget(LiftStates.INTERMEDIARY);
                }

            } break;
            case ONE: {
                setManualIntakeEnabled(true);
                setManualOuttakeEnabled(true);

                if(hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged()) {
                    outtake.setMaxManualTarget(LiftStates.HIGH);
                }

            } break;
            case TWO: {
                setManualIntakeEnabled(false);
                setManualOuttakeEnabled(true);

                if (hasManualIntakeEnableJustChanged() || hasManualOuttakeEnableJustChanged())  {
                    if (!hasGrabbedOnePixel) {
                        setManualOuttakeEnabled(false);

                        if (autoSpitAfterCollectingTwoPixels) { spitForTimeThen(timeForSpitting, AfterSpitting.STOP); }
                        else { stopIntake(); }

                        getTo(Position.INTERMEDIARY);
                        sleep(200, multithreading ? Update.NONE : Update.OUTTAKE);

                        if (autoGrabPixelsAfterCollectingTwoPixels) { pixels(PixelActions.COLLECT_GRAB, true); }
                    } else { stopIntake(); }

                    setManualOuttakeEnabled(true);
                    outtake.setMaxManualTarget(LiftStates.HIGH);

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
            setTarget(safeRotationThreshold, true, false);
            setState(OuttakeRotationStates.COLLECT);
            sleep(700, multithreading ? Update.NONE : Update.OUTTAKE);

            while (!opMode.isStarted() && !opMode.isStopRequested() && !isLiftConstrained()) { resetLift(); }

            resetLiftEncoders();
    }

    public void resetNumberOfCollectedPixelsToZero() {
        lastCollectedPixel = Pixels.NONE;
        numberOfCollectedPixels = StoredPixels.ZERO;

        hasGrabbedOnePixel = hasGrabbedTwoPixels = false;

        pixelsInPossession = new ArrayList<>();
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    private void sleep(long milliseconds, Update type) {
        long timeToContinueActions = System.currentTimeMillis() + milliseconds;

        switch (type) {
            case INTAKE: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                intake.read();
                robotInstance.clearBulkCache();
            } break;

            case OUTTAKE: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                outtake.update();
                outtake.read();
                robotInstance.clearBulkCache();
            } break;

            case ALL: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                update();
                read();
                robotInstance.clearBulkCache();
            } break;

            case SCORING_SYSTEM: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                updateLift();
                intake.read();
                outtake.read();
                robotInstance.clearBulkCache();
            } break;

            case SENSORS: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                updateSensors();
                read();
                robotInstance.clearBulkCache();
            } break;

            case NONE: while (System.currentTimeMillis() <= timeToContinueActions && !opMode.isStopRequested()) {
                opMode.idle();
            } break;
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
    }


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

    public void setMaxManualTarget(LiftStates state) {
        switch (state) {
            case INTERMEDIARY: {outtake.setMaxManualTarget(LiftStates.INTERMEDIARY); } break;
            case HIGH: { outtake.setMaxManualTarget(LiftStates.HIGH); } break;
        }
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

    public boolean isFirstPixelInTransfer() { return isFirstPixelInTransfer; }

    public boolean isSecondPixelInTransfer() { return isSecondPixelInTransfer; }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    protected boolean hasManualIntakeEnableJustChanged() { return lastManualIntakeEnabled != manualIntakeEnabled; }

    protected boolean hasManualOuttakeEnableJustChanged() { return lastManualOuttakeEnabled != manualOuttakeEnabled; }

    public boolean hasGrabbedOnePixel() { return hasGrabbedOnePixel; }

    public boolean hasGrabbedTwoPixels() { return hasGrabbedTwoPixels; }

    public boolean hasGrabbedPixels() { return hasGrabbedOnePixel || hasGrabbedTwoPixels; }



    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    /**Multi-action methods*/

    public void score(Score scoringOption) {

        switch (scoringOption) {
            case LOW: {setState(LiftStates.LOW); } break;
            case MID: { setState(LiftStates.MID); } break;
            case HIGH: {setState(LiftStates.HIGH); } break;
            default: {}
        }

        ifBugsOccur.reset();
        while (!opMode.isStopRequested() && outtake.isLiftBusy(5) && !isLiftConstrained() && ifBugsOccur.milliseconds() <= timeToWaitForSlidersCorrection) {
            sleep(1, multithreading ? Update.NONE : Update.OUTTAKE);
        }

        setState(OuttakeRotationStates.SCORE);
    }

    public void getTo(Position desiredPosition) {
        if (outtake.getRotationState() != OuttakeRotationStates.COLLECT) {
            if (outtake.getTarget() < safeRotationThreshold) {
                outtake.setTarget(safeRotationThreshold, true, false);

                ifBugsOccur.reset();
                while (!opMode.isStopRequested() && outtake.isLiftBusy(5) && !isLiftConstrained() && ifBugsOccur.milliseconds() <= timeToWaitForSlidersCorrection) {
                    sleep(1, multithreading ? Update.NONE : Update.OUTTAKE);
                }
            }

            setState(OuttakeRotationStates.COLLECT);
            sleep(timeToRotateSafelyWhenComingDown, multithreading ? Update.NONE : Update.OUTTAKE);
        }


        switch (desiredPosition) {
            case COLLECT: {
                if (!hasGrabbedOnePixel && !hasGrabbedTwoPixels) {
                    setState(OuttakeGripperStates.CLOSED);
                    sleep(timeToOpenGripper, multithreading ? Update.NONE : Update.OUTTAKE);
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

    public void pixels(PixelActions action, boolean autoGrab) {
        switch (action) {
            case GRAB: {
                setState(OuttakeGripperStates.OPEN);
                sleep(timeToOpenGripper, multithreading ? Update.NONE : Update.OUTTAKE);

                if (!autoGrab) {
                    setState(LiftStates.INTERMEDIARY);
                    sleep(200, multithreading ? Update.NONE : Update.OUTTAKE);
                }

            } break;

            case SCORE: {
                setState(OuttakeGripperStates.CLOSED);
                sleep(timeToOpenGripper, multithreading ? Update.NONE : Update.OUTTAKE);

                if (autoGetBackToIntermediary) { getTo(Position.INTERMEDIARY); }

                removePixelsUntilRemain((hasGrabbedOnePixel && numberOfCollectedPixels == StoredPixels.TWO) ? 1 : 0);

                hasGrabbedOnePixel = false;
                hasGrabbedTwoPixels = false;
                justScoredPixels = true;

            } break;

            case COLLECT_GRAB: {
                if (!hasGrabbedOnePixel() && !hasGrabbedTwoPixels()) {

                    if (numberOfCollectedPixels == StoredPixels.ONE) {
                        hasGrabbedOnePixel = true;
                        outtake.setMaxManualTarget(LiftStates.HIGH);
                    } else if (numberOfCollectedPixels == StoredPixels.TWO) {
                        hasGrabbedTwoPixels = true;
                        outtake.setMaxManualTarget(LiftStates.HIGH);
                    } else if (numberOfCollectedPixels == StoredPixels.ZERO) {
                        hasGrabbedOnePixel = false;
                        hasGrabbedTwoPixels = false;
                        outtake.setMaxManualTarget(LiftStates.INTERMEDIARY);
                    }

                    getTo(Position.COLLECT);

                    ifBugsOccur.reset();
                    while (!opMode.isStopRequested() && outtake.isLiftBusy(5) && !isLiftConstrained() && ifBugsOccur.milliseconds() <= timeToWaitForSlidersCorrection) {
                        sleep(1, multithreading ? Update.NONE : Update.OUTTAKE);
                    }

                    pixels(PixelActions.GRAB, autoGrab);

                }
            }
        }
    }


    public void manualControlLift(double value, boolean override, boolean overrideRotation) {
        if (Math.abs(value) > 0.1) {
            double sign =  (manualLiftPower % 2 == 0) ? Math.signum(value) : 1;
            int target = (int) (outtake.getCurrentPositionAverage() + toPower(value, manualLiftPower) * manualLiftSensitivity * sign);

            setTarget(target, override, overrideRotation);
        }
    }


    /*
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
     */


    public void startIntake(IntakeArmStates desiredArmState) {
        if (!hasGrabbedOnePixel)
            getTo(Position.INTERMEDIARY);

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
        sleep(time, multithreading ? Update.NONE : Update.INTAKE);

        if (action == AfterSpitting.STOP) { stopIntake(); }
            else if (action == AfterSpitting.GET_BACK) { startIntake(armState); }
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

    public void readLift() {
        outtake.read();
    }

}
