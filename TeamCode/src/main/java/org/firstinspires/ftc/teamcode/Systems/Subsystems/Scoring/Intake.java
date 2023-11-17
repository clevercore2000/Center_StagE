package org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring;

import static org.firstinspires.ftc.teamcode.Util.MotionHardware.Init.initializeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Enums;
import org.firstinspires.ftc.teamcode.Util.MotionHardware.Init;

class Intake {
    private final DcMotorEx motor;
    private final Servo left, right;

    private LinearOpMode opMode;

    private Enums.IntakeMotorStates motorState = Enums.IntakeMotorStates.STOP;
    private Enums.IntakeArmStates armState = Enums.IntakeArmStates.UP;

    private static final double servo_UP = 0.6, servo_STACK_UP = 0.92, servo_STACK_DOWN = 0.95, servo_DOWN = 0.99;
    //TODO: find actual values
    private static final double motorPower = 1;


    /**Initialize hardware*/
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;

        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        left = opMode.hardwareMap.get(Servo.class, "left_arm");
        right = opMode.hardwareMap.get(Servo.class, "right_arm");

        left.setDirection(Servo.Direction.REVERSE);

        initializeMotor(motor);
    }



    protected void setPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    /**Manual set for the intake state*/
    public void setState(Enums.IntakeMotorStates state) { motorState = state; }

    public void setState(Enums.IntakeArmStates state) { armState = state; }


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


    void update(){
        updateServo();
        updateMotor();
    }

    public Enums.IntakeArmStates getArmState() { return armState; }

    public Enums.IntakeMotorStates getMotorState() { return motorState; }

    public boolean isIntakeConstrained() { return Init.isConstrained(motor); }

}
