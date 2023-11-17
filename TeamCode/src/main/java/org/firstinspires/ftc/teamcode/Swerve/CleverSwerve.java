package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.Swerve.SwerveModule.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Swerve.SwerveModule.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.Util.Math.Transformations.Pose2d_2_Pose;
import static org.firstinspires.ftc.teamcode.Util.Math.Transformations.Pose_2_Pose2d;
import static org.firstinspires.ftc.teamcode.Util.Math.Transformations.doubleMatrix_2_doubleList;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer.Custom.CustomSwerveLocalizer;
import org.firstinspires.ftc.teamcode.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.Localizer.RR.SwerveLocalizer;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;


import java.util.Arrays;
import java.util.List;

public class CleverSwerve extends SwerveKinematics implements SwerveConstants {

    public enum Localizers {
        CUSTOM,
        IMU,
        ROADRUNNER
    }

    public static SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public static SwerveModule[] modules;

    public static CleverSwerve instance = null;

    public static Localizer localizer;
    public boolean FIELD_CENTRIC = false;
    public Localizers type = null;

    /*Wheels speeds / wheels angles*/
    double[][] ws_wa;

    private LinearOpMode opMode = null;
    private Telemetry telemetry = null;


    public static CleverSwerve getInstance(LinearOpMode opMode, Localizers type) {
        if (instance == null)
            instance = new CleverSwerve(opMode, type);
        return instance;
    }

    public CleverSwerve(LinearOpMode opMode, Localizers type) {
        this.opMode = opMode;

        frontLeftModule = new SwerveModule(opMode.hardwareMap, "FL", "FL_servo");
        frontRightModule = new SwerveModule(opMode.hardwareMap, "FR", "FR_servo");
        backLeftModule = new SwerveModule(opMode.hardwareMap, "BL", "BL_servo");
        backRightModule = new SwerveModule(opMode.hardwareMap, "BR", "BR_servo");

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        roadrunnerInit(opMode.hardwareMap);


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


        this.type = type;
        switch (this.type) {
            case IMU: { localizer = new Threaded_IMU(opMode); } break;
            case ROADRUNNER: { localizer = new SwerveLocalizer(opMode.hardwareMap); } break;
            case CUSTOM: { localizer = new CustomSwerveLocalizer(opMode.hardwareMap); } break;
            default: {}
        }
    }



    /**initialization*/
    public void setAngleDirection(List<Servo.Direction> directions) {
        for (int i = 0; i < directions.size(); i++)
            modules[i].setDirection(directions.get(i));
    }

    public void setSpeedDirection(List<DcMotorSimple.Direction> directions) {
        for (int i = 0; i < directions.size(); i++)
            modules[i].setDirection(directions.get(i));
    }




    public void joystickDrive(double joystick_x, double joystick_y, double joystick_head) {
        update();

        Pose vel;
        if (FIELD_CENTRIC)
            vel = new Pose(joystick_x, joystick_y, joystick_head).rotateWithRotationalMatrix(-localizer.getAngle(AngleUnit.RADIANS));
        else vel = new Pose(joystick_x, joystick_y, joystick_head);

        if (Math.abs(joystick_x) < 0.001 && Math.abs(joystick_y) < 0.001 && Math.abs(joystick_head) < 0.001)
            super.setLocked(true);
        else super.setLocked(false);

        ws_wa = super.robot2wheel(vel);

        for (int i = 0; i < 4; i++) { modules[i].run(ws_wa[0][i], ws_wa[1][i]); }

    }

    public void lockToPosition(Pose lockPos)
    {
        Pose2d currentPos = Pose_2_Pose2d(getPoseEstimate());
        Pose2d difference = Pose_2_Pose2d(lockPos).minus(currentPos);

        Vector2d xy = difference.vec().rotated(-currentPos.getHeading());

        double heading = Angle.normDelta(lockPos.heading) - Angle.normDelta(currentPos.getHeading());
        joystickDrive(xy.getX() * xyP, xy.getY() * xyP, heading * headingP);
    }



    /**setter methods*/
    @Override
    protected void setLockWheelsPosition(LockedWheelPositions desiredPosition) { super.setLockWheelsPosition(desiredPosition); }

    public void setFieldCentric(boolean f) { this.FIELD_CENTRIC = f; }

    public void setPoseEstimate(Pose poseEstimate) { localizer.setPositionEstimate(poseEstimate); }

    public void setTelemetry(Telemetry telemetry) { this.telemetry = telemetry; }



    /**getter methods*/
    public Pose getPoseEstimate() { return localizer.getRobotPosition(); }

    public Pose getVelocityEstimate() { return localizer.getRobotVelocity(); }

    public boolean isConstrained() {
        for (int i = 0; i < modules.length; i++)
            if (modules[i].isConstrained()) return true;

        return false;
    }



    /**math*/
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }




    /**update methods*/
    public void update() {
        localizer.update();

        DriveSignal signal = trajectorySequenceRunner.update(
                Pose_2_Pose2d(getPoseEstimate()),
                Pose_2_Pose2d(getVelocityEstimate()));
        if (signal != null && type != Localizers.IMU) setDriveSignal(signal);
    }

    public void updateDebuggingTelemetry() {
        if (telemetry != null) {
            telemetry.addData("BL: ", backLeftModule.getTargetAngle());
            telemetry.addData("FL: ", frontLeftModule.getTargetAngle());
            telemetry.addData("BR: ", backRightModule.getTargetAngle());
            telemetry.addData("FR: ", frontRightModule.getTargetAngle());

            telemetry.addData("raw BL: ", ws_wa[1][0]);
            telemetry.addData("raw FL: ", ws_wa[1][1]);
            telemetry.addData("raw FR: ", ws_wa[1][2]);
            telemetry.addData("raw BR: ", ws_wa[1][3]);

            telemetry.addData("LOCKED: ", super.isLocked());
            telemetry.addData("is flipped: ", backLeftModule.wheelFlipped);

            telemetry.update();
        }
    }


    /**roadrunner
     * ⫘⫘⫘⫘⫘⫘⫘⫘⫘⫘
     *⊱ ────── {.⋅ ♫ ⋅.} ───── ⊰
     *⫘⫘⫘⫘⫘⫘⫘⫘⫘⫘
     *⊱ ────── {.⋅ ♫ ⋅.} ───── ⊰
     * ⫘⫘⫘⫘⫘⫘⫘⫘⫘⫘
     */

    private TrajectorySequenceRunner trajectorySequenceRunner;
    private static TrajectoryFollower follower;

    private static VoltageSensor batteryVoltageSensor;

    private static final TrajectoryVelocityConstraint VELOCITY_CONSTRAINT = getVelocityConstraint(MAX_ANG_VEL, MAX_VEL);
    private static final TrajectoryAccelerationConstraint ACCELERATION_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);



    /**initialization*/
    private void roadrunnerInit(HardwareMap hardwareMap) {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)));
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }



    /**getter methods*/
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxAngularVelocity, double maxVelocity) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVelocity),
                new SwerveVelocityConstraint(maxVelocity, TRACK_WIDTH)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAcceleration) {
        return new ProfileAccelerationConstraint(maxAcceleration);
    }



    /**setter methods*/
    private void setDriveSignal(DriveSignal driveSignal) {
        double[][] ws_wa = super.robot2wheel(Pose2d_2_Pose(driveSignal.getVel()));
        double[][] was_waa = super.robot2wheel(Pose2d_2_Pose(driveSignal.getAccel()));

        List<Double> velocities = doubleMatrix_2_doubleList(ws_wa);
        List<Double> accelerations = doubleMatrix_2_doubleList(was_waa);

        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations,
                K_VELOCITY, K_ACCELERATION, K_STATIC);

        List<Double> angles = super.computeWheelAngles(ws_wa, was_waa);

        for (int i = 0; i < modules.length; i++) { modules[i].run(powers.get(i), angles.get(i)); }
    }



    /**building trajectories*/
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



    /**following trajectories*/
    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(Pose2d_2_Pose(trajectory.start()))
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }



    /**getter methods*/
    public Pose getLastError() { return Pose2d_2_Pose(trajectorySequenceRunner.getLastPoseError()); }

    public boolean isBusy() { return trajectorySequenceRunner.isBusy(); }



    public void waitForIdle() { while (!Thread.currentThread().isInterrupted() && isBusy()) { update(); } }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }


}
