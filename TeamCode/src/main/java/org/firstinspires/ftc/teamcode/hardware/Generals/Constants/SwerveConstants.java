package org.firstinspires.ftc.teamcode.hardware.Generals.Constants;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.SwerveModule.SwerveState;

public class SwerveConstants {
    /**Friction FeedForward**/
    public static double K_STATIC = 0.03; //to be tuned
    public static double K_VELOCITY = 1; //to be tuned
    public static double K_ACCELERATION = 1; //to be tuned

    public static boolean MOTOR_FLIPPING = true;
    public static boolean FIELD_CENTRIC = true;
    public static boolean USING_FEEDFORWARD = true;
    public static boolean VALUE_REJECTION = false;

    public static boolean usingAxons = false;
    public static boolean usingDriveSensitivity = false;
    public static boolean usingButtonSensitivity = false;
    public static boolean usingHeadingCorrection = false;
    public static boolean usingVelocityToggle = true;

    public static Enums.Swerve.LockedWheelPositions lockedStatus = Enums.Swerve.LockedWheelPositions.DEFAULT;

    public static final double fastConstrain = 1;
    public static final double slowConstrain = 0.4;

    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 2;
    public static double MAX_ANG_ACCEL = 2;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients MODULE_PID = new PIDCoefficients(0.6, 0, 0.1);

    public static double xyP = TRANSLATIONAL_PID.kP, headingP = HEADING_PID.kP;
    public static double moduleP = MODULE_PID.kP, angleD = MODULE_PID.kD;

    public static SwerveState calculateFeedforward(SwerveState velocities, SwerveState accelerations) {
        if (!USING_FEEDFORWARD)
            return velocities.sum(accelerations);
        return velocities.multiplyBy(K_VELOCITY).sum(accelerations.multiplyBy(K_ACCELERATION)).sum(K_STATIC);
    }
}

