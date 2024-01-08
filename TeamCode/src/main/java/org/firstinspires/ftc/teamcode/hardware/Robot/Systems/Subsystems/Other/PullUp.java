package org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other;

import static org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init.isConstrained;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;


public class PullUp implements Enums.Other {
    private final DcMotorEx motor;
    private HardwareMap hardwareMap;

    public PullUp(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, "pullup");

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

    public void setPower(double power) { motor.setPower(power); }
}
