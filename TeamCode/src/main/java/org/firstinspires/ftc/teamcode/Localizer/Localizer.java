package org.firstinspires.ftc.teamcode.Localizer;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;

/**Localizer interface for smoother code*/
public interface Localizer {
    double getAngle(AngleUnit unit);

    void update();

    Pose getRobotPosition();

    Pose getRobotVelocity();

    void setPositionEstimate(Pose newPose);

    default double fromRadiansToDegrees(double val) {
        return val * 180 / Math.PI;
    }

    default double fromDegreesToRadians(double val) {
        return val *Math.PI / 180;
    }
}
