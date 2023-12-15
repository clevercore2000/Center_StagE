package org.firstinspires.ftc.teamcode.Generals.Constants;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveState;

public class SwerveConstants {
    /**Friction FeedForward**/
    public static double K_STATIC = 0.03; //to be tuned
    public static double K_VELOCITY = 1; //to be tuned
    public static double K_ACCELERATION = 1; //to be tuned

    public static boolean FIELD_CENTRIC = false;
    public static boolean USING_FEEDFORWARD = true;
    public static boolean feelingRisky = false;

    public static Enums.Swerve.LockedWheelPositions lockedStatus = Enums.Swerve.LockedWheelPositions.DEFAULT;

    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 2;
    public static double MAX_ANG_ACCEL = 2;

    public static double xyP = 1.2, headingP = 0.7;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static SwerveState calculateFeedforward(SwerveState velocities, SwerveState accelerations) {
        if (!USING_FEEDFORWARD)
            return velocities.sum(accelerations);
        return velocities.multiplyBy(K_VELOCITY).sum(accelerations.multiplyBy(K_ACCELERATION)).sum(K_STATIC);
    }
}

