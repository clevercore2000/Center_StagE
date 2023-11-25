package org.firstinspires.ftc.teamcode.Generals;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveState;

public interface SwerveConstants {
    /**Friction FeedForward**/
    double K_STATIC = 0.03; //to be tuned
    double K_VELOCITY = 1; //to be tuned
    double K_ACCELERATION = 1; //to be tuned

    boolean FIELD_CENTRIC = false;
    boolean USING_FEEDFORWARD = true;

    double MAX_VEL = 30;
    double MAX_ACCEL = 30;
    double MAX_ANG_VEL = 2;
    double MAX_ANG_ACCEL = 2;

    double xyP = 1.2, headingP = 0.7;

    PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    static SwerveState calculateFeedforward(SwerveState velocities, SwerveState accelerations) {
        if (!USING_FEEDFORWARD)
            return velocities.sum(accelerations);
        return velocities.multiplyBy(K_VELOCITY).sum(accelerations.multiplyBy(K_ACCELERATION)).sum(K_STATIC);
    }
}

