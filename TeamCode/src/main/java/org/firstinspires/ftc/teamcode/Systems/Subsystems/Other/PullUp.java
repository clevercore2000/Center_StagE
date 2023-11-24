package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import static org.firstinspires.ftc.teamcode.Util.MotionHardware.Init.getCurrent;
import static org.firstinspires.ftc.teamcode.Util.MotionHardware.Init.isConstrained;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Generals.Enums;

public class PullUp implements Enums{
    private final DcMotorEx motor;
    private final DigitalChannel magnetic;

    private HardwareMap hardwareMap = null;

    private static double hangingEncoderPosition = -6400;
    private static double downEncoderPosition = -9700;

    private boolean hasToGetToHang = true;
    private boolean hasToGetAllTheWayDown = true;
    private boolean isReallyAllTheWayUp = false;

    private PullUpPositions position = PullUpPositions.PENDING;

    public PullUp(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        magnetic = hardwareMap.get(DigitalChannel.class, "magnetic");
        motor = hardwareMap.get(DcMotorEx.class, "pullup");

        magnetic.setMode(DigitalChannel.Mode.INPUT);
        initializeMotor(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setState(PullUpPositions position) { this.position = position; }

    public void hasToGetToHang(boolean hasToGetToHang) { this.hasToGetToHang = hasToGetToHang; }

    public void update() {

        switch (position) {
            case UP: {
                if (isAllTheWayUp()) {
                    motor.setPower(0);

                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    position = PullUpPositions.PENDING;
                    hasToGetToHang = true;
                    hasToGetAllTheWayDown = true;
                    setState(PullUpPositions.PENDING);

                } else { motor.setPower(1); }
            } break;

            case HANGING: {
                if (hasToGetToHang) {
                    if (motor.getCurrentPosition() >= hangingEncoderPosition) { motor.setPower(-1); }
                    else {
                        motor.setPower(0);
                        hasToGetToHang = false;
                        hasToGetAllTheWayDown = true;
                        setState(PullUpPositions.PENDING);
                    }
                }

            } break;

            case DOWN: {
                if (hasToGetAllTheWayDown && motor.getCurrentPosition() <= downEncoderPosition) { motor.setPower(-1); }
                else {
                    motor.setPower(0);
                    hasToGetAllTheWayDown = false;
                    setState(PullUpPositions.PENDING);
                }
            } break;
            default: {}
        }
    }

    public double getPosition() { return motor.getCurrentPosition(); }

    public PullUpPositions getState() { return position; }

    public void setPower(double power) {
        if (isAllTheWayUp() && power <= 0)
            motor.setPower(power);

        if (!isAllTheWayUp())
            motor.setPower(power);
    }

    public boolean isConstrained(DcMotorEx motor) { return getCurrent(motor) > 4; }

    public boolean isAllTheWayUp() { return magnetic.getState() ? false : true; }
}
