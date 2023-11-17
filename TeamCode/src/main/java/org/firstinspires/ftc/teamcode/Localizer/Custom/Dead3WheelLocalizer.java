package org.firstinspires.ftc.teamcode.Localizer.Custom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Math.Point;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;

/**Standard odometry class for 3 wheel localization
 * Uses an arc-based approximation of change in position
 * Basic math formulas here
 */
public class Dead3WheelLocalizer {
    /**Constants for wheel positions (relative to the robot center)*/
    private double l = 0;
    private double r = 0;
    private double b = 0;

    private double start_theta;

    public Dead3WheelLocalizer(double left_constant, double right_constant, double back_constant) {
        l = left_constant;
        r = right_constant;
        b = back_constant;
        start_theta = 0;
    }

    /**This constructor assumes that +Y direction is the front of the robot and +X the right side*/
    public Dead3WheelLocalizer(Point left_point, Point right_Point, Point back_point, double W, double L)
    {
        Point robot_center = new Point(W / 2, L / 2);
        l = Math.abs(robot_center.x - left_point.x);
        r = Math.abs(robot_center.x - right_Point.x);
        b = Math.abs(robot_center.y - back_point.y);
        start_theta = 0;
    }

    public void setStartOrientation(double start_theta) { this.start_theta = start_theta; }

    /**Returns the change in angle of the robot
     * @param delta_L: distance traveled by the left wheel
     * @param delta_R: distance traveled by the right wheel
     * @return the angle in the desired AngleUnit
     */
    private double calculateHeading(double delta_L, double delta_R, AngleUnit angleUnit)
    {
        double robot_theta = (delta_L - delta_R) / (l + r);
        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(robot_theta) : robot_theta;
    }

    /**Axis +Y in ROBOT-FRAME is the straight-line between the two positions*/
    private double calculateY(double delta_R, double theta)
    {
        double robot_y;

        if (theta == 0) robot_y = delta_R;
            else robot_y = 2 * ((theta != 0) ? delta_R / theta + r : r) * Math.sin(theta / 2);

        return robot_y;
    }

    /**Axis +X in ROBOT-FRAME**/
    private double calculateX(double delta_B, double theta) {
        double robot_x;

        if (theta == 0) robot_x = delta_B;
            else robot_x = 2 * ((theta != 0)? delta_B / theta + b : b) * Math.sin(theta / 2);

        return robot_x;
    }

    /**Finding the delta Pose between loops
     * Then converting it from ROBOT-FRAME to FIELD-FRAME with a rotation matrix
     *
     * @param delta_L = difference between current and previous left-encoder value
     * @param delta_R = difference between current and previous right-encoder value
     * @param delta_B = difference between current and previous back-encoder value
     * @param absolute_theta = absolute orientation of the robot
     *
     * @return: difference between current Pose and previous Pose
     */
    public Pose getDeltaPoseEstimate(double delta_L, double delta_R, double delta_B, double absolute_theta)
    {
        double robot_theta = calculateHeading(delta_L, delta_R, AngleUnit.RADIANS);
        double robot_x = calculateX(delta_B, robot_theta);
        double robot_y = calculateY(delta_R, robot_theta);

        double new_absolute_theta = start_theta + robot_theta;
        double delta_absolute_theta = new_absolute_theta - absolute_theta;
        double average_theta = start_theta + delta_absolute_theta / 2;

        Pose delta_global_pose = new Pose(robot_x, robot_y, new_absolute_theta).rotateWithRotationalMatrix(-average_theta);

        return delta_global_pose;
    }

}
