package org.firstinspires.ftc.teamcode.hardware.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

public class CleverData implements Enums,
        Enums.Swerve, Enums.Other, Enums.Pathing, Enums.Scoring {

    public Localizers localizer;
    public OpMode opModeType;
    public MotionPackage motionPackage;

    public GamepadKeys.Button sensitivityButton = null;
    public GamepadKeys.Trigger sensitivityTrigger = null;

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


    public CleverData addSwerveSensitivity(GamepadKeys.Button button) {
        this.sensitivityButton = button;

        SwerveConstants.usingDriveSensitivity = true;
        SwerveConstants.usingButtonSensitivity = true;

        return this;
    }

    public CleverData addSwerveSensitivity(GamepadKeys.Trigger trigger) {
        this.sensitivityTrigger = trigger;

        SwerveConstants.usingDriveSensitivity = true;
        SwerveConstants.usingButtonSensitivity = false;

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

    public CleverData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }

    public CleverData setUsingVelocityToggle(boolean flag) {
        SwerveConstants.usingVelocityToggle = flag;
        return this;
    }

    public CleverData setUsingHeadingCorrection(boolean flag) {
        SwerveConstants.usingHeadingCorrection = flag;
        return this;
    }

}
