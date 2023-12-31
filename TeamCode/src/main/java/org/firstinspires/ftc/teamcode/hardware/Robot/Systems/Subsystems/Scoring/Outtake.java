package org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Scoring;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.multithreading;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.outtake_gripper;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.outtake_left_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.outtake_right_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.outtake_rotation;
import static org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot.batteryVoltage;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.OpModes.TestOpModes.TeleOp.Tuning.PIDF_controllerTuner;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init;

public class Outtake implements Enums.Scoring {
    private CleverBot robotInstance;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo gripper;
    private final Servo rotation;

    private int leftEncoderPosition, rightEncoderPosition;
    private double leftAmperage, rightAmperage;

    private LinearOpMode opMode;

    private OuttakeGripperStates gripperStates = OuttakeGripperStates.CLOSED;
    private OuttakeRotationStates rotationStates = OuttakeRotationStates.COLLECT;

    private final double open = 0.0, closed = 0.2;
    private final double collect = 1, score = 0;
    private final int liftCOLLECT = 0, liftINTERMEDIARY = 230, liftLOW = 600, liftMID = 1200, liftHIGH = 1730;


    private double MAX_VEL = 6000, MAX_ACC = 300, MAX_DECC = -20;
    private double profileThreshold = 100;


    private PIDController controller;
    public static double p = PIDF_controllerTuner.p, d = PIDF_controllerTuner.d;
    public static double f = PIDF_controllerTuner.f;


    private int MIN = 0, MAX = liftINTERMEDIARY, RESET = -2000;
    private int target = liftINTERMEDIARY;
    public static final int safeRotationThreshold = 540;
    public static final int outOfTransferThreshold = 200;
    private double accuracyThreshold = 20;
    private LiftStates liftState = LiftStates.INTERMEDIARY;


    private boolean needsToReset = true;
    public static final double currentThreshold = 8.4;


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public Outtake(LinearOpMode opMode)
    {
        this.opMode = opMode;
        controller = new PIDController(p, 0, d);

        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, outtake_left_motor);
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, outtake_right_motor);

        gripper = opMode.hardwareMap.get(Servo.class, outtake_gripper);
        rotation = opMode.hardwareMap.get(Servo.class, outtake_rotation);

        initializeMotor(leftMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initializeMotor(rightMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public void resetEncoders() {
        Init.resetEncoders(leftMotor);
        Init.resetEncoders(rightMotor);
    }

    public void resetLift(){
        if (isLiftConstrained() && needsToReset) {
            resetEncoders();
            needsToReset = false;
        }

        if (needsToReset) {
            target = RESET;

            if (!multithreading) {
                update();
                read();
            }
        }
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public LiftStates getLiftState() { return liftState; }

    public double getLiftAmperage() { return (leftAmperage + rightAmperage) / 2; }

    public int getTarget() { return target; }

    public int getCurrentPositionAverage() { return (int)(leftEncoderPosition + rightEncoderPosition) / 2; }

    public double getLeftPosition() { return leftEncoderPosition; }

    public double getRightPosition() { return  rightEncoderPosition; }

    public OuttakeRotationStates getRotationState() { return rotationStates; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public boolean isLiftBusy() { return Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold; }

    public boolean isLiftBusy(double accuracyThreshold) { return Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold; }

    public boolean isLiftConstrained() { return leftAmperage > currentThreshold  || rightAmperage > currentThreshold; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public void setState(OuttakeGripperStates state) { gripperStates = state; updateGripper(); }

    public void setState(OuttakeRotationStates state) { rotationStates = state; updateRotation(); }

    public void setMaxManualTarget(LiftStates target) {
        switch (target) {
            case INTERMEDIARY: { MAX = liftINTERMEDIARY; } break;
            case HIGH: { MAX = liftHIGH; } break;
        }
    }

    public void setTarget(int target, boolean override) {
        this.liftState = LiftStates.UNDEFINED;

        if (!override)
            this.target = Range.clip(target, MIN, MAX);
        else this.target = target;

        if (getCurrentPositionAverage() > safeRotationThreshold + accuracyThreshold && rotationStates != OuttakeRotationStates.SCORE) {
            setState(OuttakeRotationStates.SCORE);
            setState(OuttakeGripperStates.OPEN);
        }
    }

    public void setTarget(LiftStates liftState) {
        this.liftState = liftState;

        switch (liftState) {
            case COLLECT: { target = liftCOLLECT; } break;
            case INTERMEDIARY: { target = liftINTERMEDIARY; } break;
            case LOW: { target = liftLOW; } break;
            case MID: { target = liftMID; } break;
            case HIGH: { target = liftHIGH; } break;
            default: {}
        }

    }

    public void setRobotInstance(CleverBot robotInstance) { this.robotInstance = robotInstance; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    private void updateGripper() {
        switch (gripperStates) {
            case OPEN: { gripper.setPosition(open); } break;
            case CLOSED: { gripper.setPosition(closed); } break;
            default: {}
        }
    }

    private void updateRotation() {
        switch (rotationStates) {
            case COLLECT: { rotation.setPosition(collect); } break;
            case SCORE: { rotation.setPosition(score); } break;
            default: {}
        }
    }

    public void update() {
        pid(leftMotor, leftEncoderPosition, target);
        pid(rightMotor, rightEncoderPosition, target);
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    private void pid(DcMotorEx motor, int currentPosition, int target) {
        double power = controller.calculate(currentPosition, target);

        motor.setPower(power);
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    private void initializeMotor(DcMotorEx motor, DcMotor.RunMode runMode) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */




    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public void read() {
        leftEncoderPosition = leftMotor.getCurrentPosition();
        rightEncoderPosition = rightMotor.getCurrentPosition();
        leftAmperage = leftMotor.getCurrent(CurrentUnit.AMPS);
        rightAmperage = rightMotor.getCurrent(CurrentUnit.AMPS);
    }


    public double getMAX() { return MAX; }

    public double getMIN() { return MIN; }

}
