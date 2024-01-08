package org.firstinspires.ftc.teamcode.hardware.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BL_encoder;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BL_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BL_servo;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BR_encoder;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BR_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BR_servo;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FL_encoder;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FL_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FL_servo;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FR_encoder;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FR_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FR_servo;
import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.MathFormulas.toPower;
import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.Transformations.Pose2d_2_Pose;
import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.Transformations.Pose_2_Pose2d;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Generals.Localizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.RR.SwerveLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.RR.TwoWheelSwerveLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.SwerveModule.SwerveModule;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.motion.RR.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.SwerveModule.SwerveState;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathFollowers.MotionSignal;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CleverSwerve extends SwerveKinematics implements Enums.Swerve, Enums {
    private static CleverSwerve instance;
    private CleverBot robotInstance;

    private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private List<SwerveModule> modules = new ArrayList<>();
    private SwerveState currentState;

    double batteryVoltage;
    private boolean hasLockedToPosition = false;

    private Localizer localizer;
    private Localizers localizerType;
    private MotionPackage motionPackage = MotionPackage.ROADRUNNER;

    private VoltageSensor batteryVoltageSensor;

    private LinearOpMode opMode;
    private OpMode opModeType;
    private Telemetry telemetry = null;

    private MotionSignal signal;


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public CleverSwerve getInstance() {
        return this;
    }

    public CleverSwerve(LinearOpMode opMode, Localizers localizerType, OpMode opModeType) {
        this.opMode = opMode;

        frontLeftModule = new SwerveModule(opMode.hardwareMap, FL_motor, FL_servo, FL_encoder);
        frontRightModule = new SwerveModule(opMode.hardwareMap, FR_motor, FR_servo, FR_encoder);
        backLeftModule = new SwerveModule(opMode.hardwareMap, BL_motor, BL_servo, BL_encoder);
        backRightModule = new SwerveModule(opMode.hardwareMap, BR_motor, BR_servo, BR_encoder);
        initializeModuleList();

        //TODO: reverse directions if necessary
        setSpeedDirection(Arrays.asList(
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE
        ));

        setAngleDirection(Arrays.asList(
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD
        ));

        /*setOffset(Arrays.asList(
                0.5, 0.5, 0.5, 0.5
        ), ModuleOffset.POSITION);*/

        if (motionPackage == MotionPackage.ROADRUNNER) {
            initializeRoadrunner();
        }

        this.localizerType = localizerType;
        this.opModeType = opModeType;

        switch (this.localizerType) {
            case IMU: { localizer = new Threaded_IMU(opMode); } break;
            case ROADRUNNER_THREE_WHEELS: { localizer = new SwerveLocalizer(opMode.hardwareMap); } break;
            case ROADRUNNER_TWO_WHEELS: { localizer = new TwoWheelSwerveLocalizer(opMode); } break;
            case CUSTOM: { localizer = new CustomSwerveLocalizer(opMode.hardwareMap); } break;
            default: {}
        }
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    private void initializeModuleList() {
        modules.add(backLeftModule);
        modules.add(frontLeftModule);
        modules.add(frontRightModule);
        modules.add(backRightModule);
    }

    private void initializeRoadrunner() {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)));
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public void setAngleDirection(List<Servo.Direction> directions) {
        for (int i = 0; i < directions.size() && i < modules.size(); i++)
            modules.get(i).setDirection(directions.get(i));
    }

    public void setSpeedDirection(List<DcMotorSimple.Direction> directions) {
        for (int i = 0; i < directions.size() && i < modules.size(); i++)
            modules.get(i).setDirection(directions.get(i));
    }

    public void setOffset(List<Double> offsets, ModuleOffset offsetType) {
        switch (offsetType) {
            case POSITION: {
                for (int i = 0; i < offsets.size() && i < modules.size(); i++)
                    modules.get(i).fromServoPowerToAngle(offsets.get(i).doubleValue());
            }
            break;

            case ANGLE: {
                for (int i = 0; i < offsets.size(); i++)
                    modules.get(i).setOffset(offsets.get(i).doubleValue());
            }
            break;
        }
    }

    public void setBatteryVoltageSensor(VoltageSensor batteryVoltageSensor) {
        this.batteryVoltageSensor = batteryVoltageSensor;
    }

    public void setRobotInstance(CleverBot robotInstance) {
        this.robotInstance = robotInstance;
    }



    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public void drive(double x, double y, double head, double speed) {
        update();

        Pose actualVector;
        if (FIELD_CENTRIC)
            actualVector = new Pose(x, y, head).rotateWithRotationalMatrix(-(localizer.getAngle(AngleUnit.RADIANS) + Math.PI));
        else actualVector = new Pose(x, y, head);

        if (Math.abs(actualVector.x) < 0.001 && Math.abs(actualVector.y) < 0.001 && Math.abs(actualVector.heading) < 0.001)
            super.setLocked(true);
        else super.setLocked(false);

        currentState = super.robot2moduleVelocity(actualVector);

        int i = 0;
        for (SwerveState eachState : currentState.getList()) {
            modules.get(i).run(eachState.speed * speed, eachState.angle);
            i++;
        }

    }

    double a = -0.00751709;
    double b = 0.15246;
    double c = -0.887245;
    double d = 2.14355;

    private double funkyFunction(double value) {
        double absolute = Math.abs(value);
        return (a * toPower(absolute, 4) + b * toPower(absolute, 3) + c * toPower(absolute, 2) + d * absolute) * Math.signum(value);
    }

    private double xThreshold = 3, yThreshold = 3, headingThreshold = 1.5;

    public void lockToPosition(Pose lockPosition) {
        Pose currentPosition = getPoseEstimate();
        Pose difference = lockPosition.subtract(currentPosition);

        telemetry.addLine("             DIFFERENCE"                     );
        telemetry.addData("dx:        ", difference.x                      );
        telemetry.addData("dy:        ", difference.y                      );
        telemetry.addData("dheading:  ", Math.toDegrees(difference.heading));

        difference.x = Math.abs(difference.x) <= xThreshold ? 0 : difference.x;
        difference.y = Math.abs(difference.y) <= yThreshold ? 0 : difference.y;
        difference.heading = Math.abs(Math.toDegrees(difference.heading)) <= headingThreshold ? 0 : difference.heading;

        if (difference.x == 0 && difference.y == 0 && difference.heading == 0)
            hasLockedToPosition = true;
        else hasLockedToPosition = false;

        Pose rotated_difference = difference.rotateWithRotationalMatrix(-localizer.getAngle(AngleUnit.RADIANS));

        drive(-rotated_difference.y * xyP, -rotated_difference.x * xyP, -difference.heading * headingP, fastConstrain);
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    @Override
    protected void setLockStatus(LockedWheelPositions desiredPosition) { super.setLockStatus(desiredPosition); }

    public void setMotionPackage(MotionPackage motionPackage) { this.motionPackage = motionPackage; }

    public void setPoseEstimate(Pose poseEstimate) { localizer.setPositionEstimate(poseEstimate); }

    public void setTelemetry(Telemetry telemetry) { this.telemetry = telemetry; }

    public void setPID(double p, double i, double d) {
        for (SwerveModule module : modules) { module.setPID(p, i, d); }
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */

    public Localizer getLocalizer() { return localizer; }

    public Pose getPoseEstimate() { return localizer.getRobotPosition(); }

    public double getAngle() { return localizer.getAngle(AngleUnit.DEGREES); }

    public Pose getVelocityEstimate() { return localizer.getRobotVelocity(); }

    public double getBatteryVoltage() { return batteryVoltage; }

    public boolean isConstrained() {
        for (int i = 0; i < modules.size(); i++)
            if (modules.get(i).isConstrained()) return true;

        return false;
    }

    public void read() {
        if (usingAxons)
            for (SwerveModule module: modules) { module.read(); }
        if (localizer instanceof CustomSwerveLocalizer)
            localizer.read();
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public static double rpmToVelocity(double rpm) {
        return rpm * SwerveModule.GEAR_RATIO * 2 * Math.PI * SwerveModule.WHEEL_RADIUS / 60.0;
    }

    public void resetAngle() { if (localizer instanceof Threaded_IMU)
        ((Threaded_IMU) localizer).reset();
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public void update() {
        localizer.update();

        if (opModeType == OpMode.AUTONOMUS) { updateAutonomus(); }
    }

    public void updateAutonomus() {
        if (motionPackage == MotionPackage.ROADRUNNER) {

            DriveSignal signal = trajectorySequenceRunner.update(
                    Pose_2_Pose2d(getPoseEstimate()),
                    Pose_2_Pose2d(getVelocityEstimate()));
            if (signal != null && localizerType != Localizers.IMU) setDriveSignal(signal);

        } else if (motionPackage == MotionPackage.CUSTOM) {

            if (signal != null) {
                if (USING_FEEDFORWARD) {
                    signal.velocity = new Pose(signal.velocity.multiplyBy(K_VELOCITY).getPoint().sum(K_STATIC),
                            signal.velocity.multiplyBy(K_VELOCITY).heading);
                }

                drive(signal.velocity.x, signal.velocity.y, signal.velocity.heading, fastConstrain);
            }
        }
    }

    public void updateDebuggingTelemetry() {
        if (telemetry != null) {
            /*telemetry.addData("BL: ", backLeftModule.getModuleTarget());
            telemetry.addData("FL: ", frontLeftModule.getModuleTarget());
            telemetry.addData("BR: ", backRightModule.getModuleTarget());
            telemetry.addData("FR: ", frontRightModule.getModuleTarget());

            telemetry.addData("raw BL: ", currentState.get(0, state.ANGLE));
            telemetry.addData("raw FL: ", currentState.get(1, state.ANGLE));
            telemetry.addData("raw FR: ", currentState.get(2, state.ANGLE));
            telemetry.addData("raw BR: ", currentState.get(3, state.ANGLE));*/

            telemetry.addData("LOCKED: ", super.isLocked());
            telemetry.addData("is flipped: ", backLeftModule.wheelFlipped);
            telemetry.addData("Angle: ", getAngle());

            telemetry.update();
        }
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */

    /**CUSTOM*/
    public void setMotionSignal(MotionSignal signal) { this.signal = signal; }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    /**ROADRUNNER*/
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private static TrajectoryFollower follower;

    private static final TrajectoryVelocityConstraint VELOCITY_CONSTRAINT = getVelocityConstraint(MAX_ANG_VEL, MAX_VEL);
    private static final TrajectoryAccelerationConstraint ACCELERATION_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxAngularVelocity, double maxVelocity) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVelocity),
                new SwerveVelocityConstraint(maxVelocity, BASE_WIDTH)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAcceleration) {
        return new ProfileAccelerationConstraint(maxAcceleration);
    }

    public Pose getLastError() { return Pose2d_2_Pose(trajectorySequenceRunner.getLastPoseError()); }


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */


    private void setDriveSignal(DriveSignal driveSignal) {
        SwerveState velocities = super.robot2moduleVelocity(Pose2d_2_Pose(driveSignal.getVel()));
        SwerveState accelerations = super.robot2moduleAcceleration(Pose2d_2_Pose(driveSignal.getAccel()));

        SwerveState powers = SwerveConstants.calculateFeedforward(velocities, accelerations);

        for (int i = 0; i < modules.size(); i++) { modules.get(i).run(powers.get(i, state.SPEED), powers.get(i, state.ANGLE)); }
    }


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */


    public TrajectoryBuilder trajectoryBuilder(Pose startPose) {
        return new TrajectoryBuilder(Pose_2_Pose2d(startPose), VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose startPose, boolean inverseTrajectory) {
        return new TrajectoryBuilder(Pose_2_Pose2d(startPose), inverseTrajectory, VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose startPose, double startHeading) {
        return new TrajectoryBuilder(Pose_2_Pose2d(startPose), startHeading, VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose startPose) {
        return new TrajectorySequenceBuilder(
                Pose_2_Pose2d(startPose),
                VELOCITY_CONSTRAINT, ACCELERATION_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */


    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */



    public boolean isBusy() {
        return motionPackage == MotionPackage.PID ? !hasLockedToPosition : false; /*:
                (trajectorySequenceRunner != null) ? trajectorySequenceRunner.isBusy() : false;*/
    }

    public void waitForIdle() { while (!Thread.currentThread().isInterrupted() && isBusy()) { update(); } }


    /**
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */


    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }


}
