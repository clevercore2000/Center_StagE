package org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other;

import static org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init.isConstrained;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

@Deprecated
@Disabled
public class PullUp implements Enums.Other {
    private final DcMotorEx motor;
    private final DigitalChannel magnetic;

    private HardwareMap hardwareMap;

    private static double hangingEncoderPosition = -5000;
    private static double downEncoderPosition = -7300;

    private boolean hasReset = false;
    private boolean hasToGetToHang = true;
    private boolean hasToGetAllTheWayDown = true;
    private boolean isReallyAllTheWayUp = false;

    private PullUpPositions position = PullUpPositions.PENDING;
    private PullUpPositions lastPosition = PullUpPositions.PENDING;

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
                    hasReset = true;
                    setState(PullUpPositions.PENDING);

                } else { motor.setPower(1); }
            } break;

            case HANGING: {
                if (hasToGetToHang) {
                    if (encoderPosition >= hangingEncoderPosition) { motor.setPower(-1); }
                    else {
                        motor.setPower(0);
                        hasToGetToHang = false;
                        hasToGetAllTheWayDown = true;
                        setState(PullUpPositions.PENDING);
                    }
                }

            } break;

            case DOWN: {
                if (hasToGetAllTheWayDown && encoderPosition >= downEncoderPosition && hasReset) { motor.setPower(-1); }
                else {
                    motor.setPower(0);
                    hasToGetAllTheWayDown = false;
                    setState(PullUpPositions.PENDING);
                }
            } break;
            default: {}
        }
    }

    public double getPosition() { return encoderPosition; }

    public PullUpPositions getState() { return position; }

    public void setPower(double power) {
        if (isAllTheWayUp() && power <= 0)
            motor.setPower(power);

        if (!isAllTheWayUp())
            motor.setPower(power);
    }

    public boolean isAllTheWayUp() { return magnetic.getState() ? false : true; }

    private double encoderPosition;

    public void read() { encoderPosition = motor.getCurrentPosition(); }
}
