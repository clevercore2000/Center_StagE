package org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring;

import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.intake_left_servo;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.intake_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.intake_right_servo;
import static org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init.initializeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

public class Intake implements Enums.Scoring {
    private final DcMotorEx motor;
    private final Servo left, right;

    private double intakeAmperage;

    private LinearOpMode opMode;

    private IntakeMotorStates motorState = IntakeMotorStates.STOP;
    private IntakeArmStates armState = IntakeArmStates.UP;

    private static final double servo_UP = 0.6, servo_STACK_UP = 0.92, servo_STACK_DOWN = 0.95, servo_DOWN = 0.99;
    private static final double motorPower = 1;
    private static final double currentThreshold = 6;


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    /**Initialize hardware*/
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;

        motor = opMode.hardwareMap.get(DcMotorEx.class, intake_motor);
        left = opMode.hardwareMap.get(Servo.class, intake_left_servo);
        right = opMode.hardwareMap.get(Servo.class, intake_right_servo);

        left.setDirection(Servo.Direction.REVERSE);

        initializeMotor(motor);
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    protected void setPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    /**Manual set for the intake state*/
    public void setState(IntakeMotorStates state) { motorState = state; updateMotor(); }

    public void setState(IntakeArmStates state) { armState = state; updateServo(); }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    private void updateMotor() {
        switch (motorState) {
            case COLLECT: { motor.setPower(-motorPower); } break;
            case SPIT: { motor.setPower(motorPower); } break;
            case STOP: { motor.setPower(0); } break;
            default: {}
        }
    }

    private void updateServo() {
        switch (armState) {
            case UP: { setPosition(servo_UP); } break;
            case STACK_UP: { setPosition(servo_STACK_UP); } break;
            case STACK_DOWN: { setPosition(servo_STACK_DOWN); } break;
            case DOWN: { setPosition(servo_DOWN); }
            default: {}
        }
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public IntakeArmStates getArmState() { return armState; }

    public IntakeMotorStates getMotorState() { return motorState; }

    public double getIntakeAmperage() { return intakeAmperage; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public boolean isIntakeConstrained() { return intakeAmperage > currentThreshold; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public void read() { intakeAmperage = motor.getCurrent(CurrentUnit.AMPS); }

}
