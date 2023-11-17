package org.firstinspires.ftc.teamcode.Localizer.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;

/**Test class for field-centric drive before mounting odometry wheels
 * Used strictly for the HEADING value**/
public class Threaded_IMU implements Localizer {
    public static IMU imu;
    public static Threaded_IMU instance = null;

    private Thread imuThread;
    private final Object imuAngleLock = new Object();
    private final Object imuVelocityLock = new Object();

    private double imuAngle = 0.0;
    private double imuAngularVelocity = 0.0;
    private double imuOffset = 0.0;

    private final RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

    public static Threaded_IMU getInstance(LinearOpMode opMode) {
        if (instance == null) { instance = new Threaded_IMU(opMode); }

        return instance;
    }

    public Threaded_IMU(LinearOpMode opMode) {
        synchronized (imuAngleLock) {
            imu = opMode.hardwareMap.get(IMU.class, "imu");

            IMU.Parameters parameters = new IMU.Parameters(imuOrientation);
            imu.initialize(parameters);

            reset();
            startIMUThread(opMode);
        }
    }

    private void startIMUThread(LinearOpMode opMode) {

            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {

                    synchronized (imuAngleLock) {
                        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }

                    synchronized (imuVelocityLock) {
                        imuAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate; }
                }
            });

            imuThread.start();
    }

    /**The only useful value from here**/
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuAngle + imuOffset; }
            case DEGREES: { return fromRadiansToDegrees(imuAngle + imuOffset); }
            default: { return 0; }
        }
    }

    //TODO: find the unit measure
    public double getAngularVelocity() { return imuAngularVelocity; }

    public Pose getRobotPosition() { return new Pose(0, 0, getAngle(AngleUnit.DEGREES)); }

    public Pose getRobotVelocity() { return new Pose(); }



    public void update() {}

    public void reset() { synchronized (imuAngleLock) { imu.resetYaw(); } }

    public void setPositionEstimate(Pose newPoseEstimate) { imuOffset = newPoseEstimate.heading - imuAngle; }
}
