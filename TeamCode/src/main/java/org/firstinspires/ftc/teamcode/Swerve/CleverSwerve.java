package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveModule.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveModule.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.WayFinder.Math.Transformations.Pose2d_2_Pose;
import static org.firstinspires.ftc.teamcode.WayFinder.Math.Transformations.Pose_2_Pose2d;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Generals.SwerveConstants;
import org.firstinspires.ftc.teamcode.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.Localizer.RR.SwerveLocalizer;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveState;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathFollowers.MotionSignal;


import java.util.Arrays;
import java.util.List;

public class CleverSwerve extends SwerveKinematics implements SwerveConstants, Enums.Swerve, Enums {
    private static CleverSwerve instance;

    private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private List<SwerveModule> modules;
    private SwerveState currentState;

    private Localizer localizer;
    private Localizers localizerType;
    private MotionPackage motionPackage = MotionPackage.CUSTOM;

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


    public static CleverSwerve getInstance(LinearOpMode opMode, Localizers localizerType, OpMode opModeType  ) {
        instance = new CleverSwerve(opMode, localizerType, opModeType);
        return instance;
    }

    public CleverSwerve(LinearOpMode opMode, Localizers localizerType, OpMode opModeType) {
        this.opMode = opMode;
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        frontLeftModule = new SwerveModule(opMode.hardwareMap, "FL", "FL_servo");
        frontRightModule = new SwerveModule(opMode.hardwareMap, "FR", "FR_servo");
        backLeftModule = new SwerveModule(opMode.hardwareMap, "BL", "BL_servo");
        backRightModule = new SwerveModule(opMode.hardwareMap, "BR", "BR_servo");
        initializeModuleList();

        //TODO: reverse directions if necessary
        setSpeedDirection(Arrays.asList(
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD
        ));

        setAngleDirection(Arrays.asList(
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD,
                Servo.Direction.FORWARD
        ));

        if (motionPackage == MotionPackage.ROADRUNNER) { initializeRoadrunner(); }

        this.localizerType = localizerType;
        this.opModeType = opModeType;

        switch (this.localizerType) {
            case IMU: { localizer = new Threaded_IMU(opMode); } break;
            case ROADRUNNER: { localizer = new SwerveLocalizer(opMode.hardwareMap); } break;
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
        modules.add(frontLeftModule);
        modules.add(frontRightModule);
        modules.add(backRightModule);
        modules.add(backLeftModule);
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
        for (int i = 0; i < directions.size(); i++)
            modules.get(i).setDirection(directions.get(i));
    }

    public void setSpeedDirection(List<DcMotorSimple.Direction> directions) {
        for (int i = 0; i < directions.size(); i++)
            modules.get(i).setDirection(directions.get(i));
    }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public void drive(double x, double y, double head) {
        update();

        Pose actualVector;
        if (FIELD_CENTRIC)
            actualVector = new Pose(x, y, head).rotateWithRotationalMatrix(-localizer.getAngle(AngleUnit.RADIANS));
        else actualVector = new Pose(x, y, head);

        if (Math.abs(actualVector.x) < 0.001 && Math.abs(actualVector.y) < 0.001 && Math.abs(actualVector.heading) < 0.001)
            super.setLocked(true);
        else super.setLocked(false);

        currentState = super.robot2moduleVelocity(actualVector);

        int i = 0;
        for (SwerveState eachState : currentState.getList()) { modules.get(i).run(eachState.speed, eachState.angle); i++; }

    }

    public void lockToPosition(Pose lockPosition)
    {
        Pose currentPosition = getPoseEstimate();
        Pose difference = lockPosition.subtract(currentPosition);

        Pose rotated_difference = difference.rotateWithRotationalMatrix(-currentPosition.heading);
        drive(rotated_difference.x * xyP, rotated_difference.y * xyP, difference.heading * headingP);
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


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */

    public Localizer getLocalizer() { return localizer; }

    public Pose getPoseEstimate() { return localizer.getRobotPosition(); }

    public Pose getVelocityEstimate() { return localizer.getRobotVelocity(); }

    public boolean isConstrained() {
        for (int i = 0; i < modules.size(); i++)
            if (modules.get(i).isConstrained()) return true;

        return false;
    }

    public void read() { localizer.read(); }


    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */


    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
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

                drive(signal.velocity.x, signal.velocity.y, signal.velocity.heading);
            }
        }
    }

    public void updateDebuggingTelemetry() {
        if (telemetry != null) {
            telemetry.addData("BL: ", backLeftModule.getTargetAngle());
            telemetry.addData("FL: ", frontLeftModule.getTargetAngle());
            telemetry.addData("BR: ", backRightModule.getTargetAngle());
            telemetry.addData("FR: ", frontRightModule.getTargetAngle());

            telemetry.addData("raw BL: ", currentState.get(0, state.ANGLE));
            telemetry.addData("raw FL: ", currentState.get(1, state.ANGLE));
            telemetry.addData("raw FR: ", currentState.get(2, state.ANGLE));
            telemetry.addData("raw BR: ", currentState.get(3, state.ANGLE));

            telemetry.addData("LOCKED: ", super.isLocked());
            telemetry.addData("is flipped: ", backLeftModule.wheelFlipped);

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



    public boolean isBusy() { return trajectorySequenceRunner.isBusy(); }

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
