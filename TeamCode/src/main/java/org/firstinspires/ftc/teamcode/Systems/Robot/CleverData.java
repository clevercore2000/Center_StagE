package org.firstinspires.ftc.teamcode.Systems.Robot;

import org.firstinspires.ftc.teamcode.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Generals.Enums;

public class CleverData implements Enums,
        Enums.Swerve, Enums.Other, Enums.Pathing, Enums.Scoring {

    public Localizers localizer;
    public OpMode opModeType;
    public MotionPackage motionPackage;


    public CleverData add(Localizers localizer) {
        this.localizer = localizer;
        return this;
    }

    public CleverData add(OpMode opModeType) {
        this.opModeType = opModeType;
        return this;
    }

    public CleverData add(MotionPackage motionPackage) {
        this.motionPackage = motionPackage;
        return this;
    }


    public CleverData setAutoSpit(boolean flag) {
        SystemConstants.autoSpitAfterCollectingTwoPixels = flag;
        return this;
    }

    public CleverData setAutoGrabPixels(boolean flag) {
        SystemConstants.autoGrabPixelsAfterCollectingTwoPixels = flag;
        return this;
    }

    public CleverData setAutoGetToIntermediary(boolean flag) {
        SystemConstants.autoGetBackToIntermediary = flag;
        return this;
    }

    public CleverData setAutoReset(boolean flag) {
        SystemConstants.autoResetSystem = flag;
        return this;
    }

    public CleverData setUsingManualEnabling(boolean flag) {
        SystemConstants.useManualEnable = flag;
        return this;
    }

    public CleverData allowOtherUsageBeforeEndgame(boolean flag) {
        SystemConstants.allowOtherSystemUsageBeforeEndgame = flag;
        return this;
    }

    public CleverData setFieldCentric(boolean flag) {
        SwerveConstants.FIELD_CENTRIC = flag;
        return this;
    }

    public CleverData setUsingFeedforward(boolean flag) {
        SwerveConstants.USING_FEEDFORWARD = flag;
        return this;
    }

    public CleverData setModuleOptimization(boolean flag) {
        SwerveConstants.feelingRisky = flag;
        return this;
    }

    public CleverData setLockedWheelStyle(LockedWheelPositions position) {
        SwerveConstants.lockedStatus = position;
        return this;
    }

    public CleverData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public CleverData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public CleverData setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public CleverData setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

}
