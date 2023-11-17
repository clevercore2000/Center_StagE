package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

interface SwerveConstants {
    /**Friction FeedForward**/
    double K_STATIC = 0.03; //to be tuned
    double K_VELOCITY = 1; //to be tuned
    double K_ACCELERATION = 1; //to be tuned

    double MAX_VEL = 30;
    double MAX_ACCEL = 30;
    double MAX_ANG_VEL = 2;
    double MAX_ANG_ACCEL = 2;

    double xyP = 1, headingP = 1;

    PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
}
