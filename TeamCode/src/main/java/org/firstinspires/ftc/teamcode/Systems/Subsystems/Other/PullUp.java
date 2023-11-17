package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import static org.firstinspires.ftc.teamcode.Util.MotionHardware.Init.isConstrained;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Enums;

class PullUp {
    private final DcMotorEx motor;
    private final RevTouchSensor touch;

    private HardwareMap hardwareMap = null;
    private ElapsedTime timerForGettingToHang;

    private double timeForGettingToHang = 3000; //ms

    private boolean hasToGetToHang = true;
    private boolean hasToGetAllTheWayDown = true;

    private Enums.PullUpPositions position;

    public PullUp(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        touch = hardwareMap.get(RevTouchSensor.class, "touch");
        motor = hardwareMap.get(DcMotorEx.class, "pullup");
        initializeMotor(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**initialization*/
    private void initializeMotor(DcMotor.RunMode runMode) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setState(Enums.PullUpPositions position) { this.position = position; }

    public void hasToGetToHang(boolean hasToGetToHang) { this.hasToGetToHang = hasToGetToHang; }

    public void update() {
        boolean isTouchPressed = touch.isPressed();

        switch (position) {
            case UP: {
                motor.setPower(1);

                if (isTouchPressed) { motor.setPower(0); }
            } break;

            case HANGING: {
                if (hasToGetToHang) {
                    timerForGettingToHang.reset();
                    hasToGetToHang(false);
                }

                if ((timerForGettingToHang.milliseconds() <= timeForGettingToHang)) { motor.setPower(-1); }
                    else { motor.setPower(0); }
            } break;

            case DOWN: {
                if (hasToGetAllTheWayDown) { motor.setPower(-1); }

                if (isConstrained(motor)) {
                    motor.setPower(0);

                    hasToGetAllTheWayDown = false;
                }
            } break;
            default: {}
        }
    }
}
