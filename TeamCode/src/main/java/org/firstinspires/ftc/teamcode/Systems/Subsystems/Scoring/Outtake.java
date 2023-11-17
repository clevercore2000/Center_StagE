package org.firstinspires.ftc.teamcode.Systems.Subsystems.Scoring;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.infinity;
import static org.firstinspires.ftc.teamcode.Util.MotionHardware.Init.isConstrained;

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
import org.firstinspires.ftc.teamcode.Systems.Enums;
import org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Tuning.PIDF_controllerTuner;
import org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling.ProfileConstrains;
import org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling.Trapezoidal.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Util.MotionHardware.Init;

class Outtake {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo gripper;
    private final Servo rotation;

    private LinearOpMode opMode;

    private Enums.OuttakeGripperStates gripperStates = Enums.OuttakeGripperStates.CLOSED;
    private Enums.OuttakeRotationStates rotationStates = Enums.OuttakeRotationStates.COLLECT;

    public static final double ticks_in_degree = 36 / 14;
    private final double open = 0.0, closed = 0.2;
    private final double collect = 0.06, score = 0.40;
    private final int liftCOLLECT = 0, liftINTERMEDIARY = 95, liftLOW = 300, liftMID = 530, liftHIGH = 875;

    private MotionProfile motionProfile;
    private ElapsedTime profileTimer;
    private double MAX_VEL = 6000, MAX_ACC = 10, MAX_DECC = 5;
    private double profileThreshold = 100;


    private PIDController controller;

    public static double p = PIDF_controllerTuner.p, d = PIDF_controllerTuner.d;
    public static double f = PIDF_controllerTuner.f;

    private int MIN = 0, MAX = 875, RESET = -2000;
    private int old_target = 0;
    private int target = liftINTERMEDIARY;
    private double accuracyThreshold = 20;

    private Enums.LiftStates liftState = Enums.LiftStates.INTERMEDIARY;

    private static boolean needsToReset = true;
    private boolean isAscending = true;


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



    public void resetEncoders() {
        Init.resetEncoders(leftMotor);
        Init.resetEncoders(rightMotor);
    }

    public void resetLift(){
        if ((isConstrained(leftMotor) || isConstrained(rightMotor)) && needsToReset) {
            resetEncoders();
            needsToReset = false;
        }

        if (needsToReset) {
            target = RESET;
            updateLift();
        }
    }

    public void setTarget(int target) {
        this.liftState = Enums.LiftStates.UNDEFINED;
        old_target = this.target;

        Range.clip(target, MIN, MAX);
        this.target = target;

    }

    public void setTarget(Enums.LiftStates liftState) {
        this.liftState = liftState;
        old_target = target;

        switch (liftState) {
            case COLLECT: { target = liftCOLLECT; } break;
            case INTERMEDIARY: { target = liftINTERMEDIARY; } break;
            case LOW: { target = liftLOW; } break;
            case MID: { target = liftMID; } break;
            case HIGH: { target = liftHIGH; } break;
            default: {}


            motionProfile = new TrapezoidalMotionProfile(getCurrentPositionAverage(), target, new ProfileConstrains(MAX_VEL, MAX_ACC));
            profileTimer.reset();

        }

    }

    public double getProfileVelocity() {
        return (motionProfile != null) ? motionProfile.calculate(profileTimer.milliseconds()).get(MotionState.val.VELOCITY) : 0; }

    public Enums.LiftStates getLiftState() { return liftState; }

    public double getLiftAmperage() { return (leftMotor.getCurrent(CurrentUnit.AMPS) + rightMotor.getCurrent(CurrentUnit.AMPS)) / 2; }

    public int getTarget() { return target; }

    int getCurrentPositionAverage() { return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2; }

    public boolean isLiftBusy() { return (Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold) ? true : false; }

    public boolean isLiftBusy(double accuracyThreshold) { return (Math.abs(getCurrentPositionAverage() - target) > accuracyThreshold) ? true : false; }

    public boolean isLiftConstrained() { return isConstrained(leftMotor) || isConstrained(rightMotor); }

    private boolean isLiftAscending() { return isAscending; }


    public void setState(Enums.OuttakeGripperStates state) { gripperStates = state; }

    public void setState(Enums.OuttakeRotationStates state) { rotationStates = state; }

    public void setLiftAscending(boolean isAscending) { this.isAscending = isAscending; }


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
        pid(leftMotor, leftMotor.getCurrentPosition(), target);
        pid(rightMotor, rightMotor.getCurrentPosition(), target);

    }

    void update() {
        updateLift();
        updateGripper();
        updateRotation();
    }


    private void pid(DcMotorEx motor, int currentPosition, int target) {
        double power = controller.calculate(currentPosition, target);

        if (motionProfile != null)
            if (motor.isBusy()) {
                MotionState currentState = motionProfile.calculate(profileTimer.milliseconds());

                double profilePower = profileVelocityToMotorInput(currentState);
                power = Math.min(power, profilePower);
            }

        motor.setPower(power);
    }

    private void initializeMotor(DcMotorEx motor, DcMotor.RunMode runMode) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double profileVelocityToMotorInput(MotionState state) {
        double velocity = state.get(MotionState.val.VELOCITY);
        double max_velocity = motionProfile.getConstrains().MAX_VELOCITY;

        return velocity / max_velocity;
    }

}
