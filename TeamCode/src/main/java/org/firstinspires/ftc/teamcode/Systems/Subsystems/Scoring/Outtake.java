package org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Tuning.PIDF_controllerTuner;
import org.firstinspires.ftc.teamcode.Generals.MotionProfile;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.ProfileConstrains;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.Trapezoidal.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Util.MotionHardware.Init;

public class Outtake implements Enums.Scoring {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo gripper;
    private final Servo rotation;

    private int leftEncoderPosition, rightEncoderPosition;
    private double leftAmperage, rightAmperage;

    private LinearOpMode opMode;

    private OuttakeGripperStates gripperStates = OuttakeGripperStates.CLOSED;
    private OuttakeRotationStates rotationStates = OuttakeRotationStates.COLLECT;

    public static final double ticks_in_degree = 36 / 14;
    private final double open = 0.0, closed = 0.2;
    private final double collect = 0.32, score = 1;
    private final int liftCOLLECT = 0, liftINTERMEDIARY = 95, liftLOW = 300, liftMID = 530, liftHIGH = 875;


    private MotionProfile motionProfile;
    private ElapsedTime profileTimer;
    private double MAX_VEL = 6000, MAX_ACC = 300, MAX_DECC = -20;
    private double profileThreshold = 100;


    private PIDController controller;
    public static double p = PIDF_controllerTuner.p, d = PIDF_controllerTuner.d;
    public static double f = PIDF_controllerTuner.f;


    private int MIN = 0, MAX = liftINTERMEDIARY, RESET = -2000;
    private int target = liftINTERMEDIARY;
    public static final int safeRotationThreshold = 290;
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

        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "rightOuttake");

        gripper = opMode.hardwareMap.get(Servo.class, "gripper");
        rotation = opMode.hardwareMap.get(Servo.class, "rotation");

        initializeMotor(leftMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initializeMotor(rightMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        profileTimer = new ElapsedTime();
        profileTimer.startTime();
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
            updateLift();
            read();
        }
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public double getProfileVelocity() {
        return (motionProfile != null) ? motionProfile.calculate(profileTimer.milliseconds()).get(MotionState.val.VELOCITY) : 0; }

    public LiftStates getLiftState() { return liftState; }

    public double getLiftAmperage() { return (leftAmperage + rightAmperage) / 2; }

    public int getTarget() { return target; }

    public int getCurrentPositionAverage() { return (int)(getLeftPosition() + getRightPosition()) / 2; }

    public double getLeftPosition() { return leftMotor.getCurrentPosition(); }

    public double getRightPosition() { return  rightMotor.getCurrentPosition(); }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    public boolean isLiftBusy() { return (Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold) ? true : false; }

    public boolean isLiftBusy(double accuracyThreshold) { return (Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold) ? true : false; }

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

    public void setTarget(int target) {
        this.liftState = LiftStates.UNDEFINED;

        this.target = Range.clip(target, MIN, MAX);
        if (target >= safeRotationThreshold) {
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


            //motionProfile = new TrapezoidalMotionProfile(getCurrentPositionAverage(), target, new ProfileConstrains(MAX_VEL, MAX_ACC));
            //profileTimer.reset();

        }

    }


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

    private void updateLift() {
        pid(leftMotor, leftEncoderPosition, target);
        pid(rightMotor, rightEncoderPosition, target);

    }

    public void update() {
        updateLift();
        //updateGripper();
        //updateRotation();
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
    */


    private void pid(DcMotorEx motor, int currentPosition, int target) {
        double power = controller.calculate(currentPosition, target);

        /*if (motionProfile != null)
            if (motor.isBusy()) {
                MotionState currentState = motionProfile.calculate(profileTimer.time());

                double profilePower = profileVelocityToMotorInput(currentState);
                power = Math.min(power, profilePower);
            }*/

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


    @Deprecated
    private double profileVelocityToMotorInput(MotionState state) {
        double velocity = state.get(MotionState.val.VELOCITY);
        double max_velocity = motionProfile.getConstrains().MAX_VELOCITY;

        return velocity / max_velocity;
    }


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
