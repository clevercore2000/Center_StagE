package org.firstinspires.ftc.teamcode.Swerve;

import org.firstinspires.ftc.teamcode.Util.Math.Point;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;

import java.util.Arrays;
import java.util.List;

abstract class SwerveKinematics {
    public enum LockedWheelPositions{
        DIAMOND,
        X_SHAPE,
        DEFAULT
    }

    private enum Component {
        X,
        Y
    }

    /**Robot-specific parameters*/
    public static double TRACK_WIDTH = 13, WHEEL_BASE = 13; //to be tuned
    private final double R = Math.hypot(TRACK_WIDTH, WHEEL_BASE) / 2;

    protected boolean locked = false;
    private LockedWheelPositions lockedStatus = LockedWheelPositions.DIAMOND;

    private double[] lastWheelAngles;

    /**Performing inverse kinematics to determine each module's state from a drivetrain state
     * #1st order
     */
    public double[][] robot2wheel(Pose p) {
        double x = p.x, y = p.y, heading = p.heading;
        double[][] ws_wa;

        double a = x - heading * (WHEEL_BASE / R),
                b = x + heading * (WHEEL_BASE / R),
                c = y - heading * (TRACK_WIDTH / R),
                d = y + heading * (TRACK_WIDTH / R);

        if (locked) { ws_wa = getLockedWheelsPosition(); }
        else
        {
            ws_wa = new double[][]{
                    {Math.hypot(b, c), Math.hypot(b, d), Math.hypot(a, d), Math.hypot(a, c)},
                    {Math.atan2(b, c), Math.atan2(b, d), Math.atan2(a, d), Math.atan2(a, c)}
            };
        }


        for (int i = 0; i < 4; i++) { lastWheelAngles[i] = ws_wa[1][i]; }

        return normalizeSpeeds(ws_wa);
    }

    /**Performing forward kinematics to determine drivetrain's state from module states*/
    public Pose wheel2robot(double[][] ws_wa) {
        double[][] x_y = new double[][]{
                {ws_wa[0][0] * Math.cos(ws_wa[1][0]), ws_wa[0][1] * Math.cos(ws_wa[1][1]), ws_wa[0][2] * Math.cos(ws_wa[1][2]), ws_wa[0][3] * Math.cos(ws_wa[1][3])},
                {ws_wa[0][0] * Math.sin(ws_wa[1][0]), ws_wa[0][1] * Math.sin(ws_wa[1][1]), ws_wa[0][2] * Math.sin(ws_wa[1][2]), ws_wa[0][3] * Math.sin(ws_wa[1][3])}
        };

        Point vectorPeak = new Point(x_y[0][0] + x_y[0][1] + x_y[0][2] + x_y[0][3],
                x_y[1][0] + x_y[1][1] + x_y[1][2] + x_y[1][3]);

        return new Pose(vectorPeak, Math.atan2(vectorPeak.y, vectorPeak.x));
    }

    private double[][] normalizeSpeeds(double[][] ws_wa) {
        double max = Math.max(ws_wa[0][0],ws_wa[0][1]);
        max = Math.max(ws_wa[0][2], max);
        max = Math.max(ws_wa[0][3], max);

        if (max > 1)
            for (int i = 0; i < 4; i ++) ws_wa[0][i] /= max;

        return ws_wa;
    }


    private double[][] getLockedWheelsPosition() {

        switch (lockedStatus) {
            case DIAMOND: {
                return new double[][]{
                        {0, 0, 0, 0},
                        {-Math.PI / 4, Math.PI / 4, -Math.PI / 4, Math.PI / 4}
                };
            }

            case X_SHAPE: {
                return new  double[][]{
                        {0, 0, 0, 0},
                        {Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4}
                };
            }

            case DEFAULT: {
                return  new double[][]{
                        {0, 0, 0, 0},
                        {lastWheelAngles[0], lastWheelAngles[1], lastWheelAngles[2], lastWheelAngles[3]}
                };
            }

            default: {
                return  new double[][]{
                    {0, 0, 0, 0},
                    {0, 0, 0, 0}
                };
            }
        }

    }

    protected List<Double> computeWheelAngles(double[][] velocity, double[][] acceleration) {
        return Arrays.asList(
                Math.atan2(getVectorComponent(velocity[0][0], acceleration[1][0], Component.Y),
                                getVectorComponent(velocity[0][0], acceleration[1][0], Component.X)),

                Math.atan2(getVectorComponent(velocity[0][1], acceleration[1][1], Component.Y),
                        getVectorComponent(velocity[0][1], acceleration[1][1], Component.X)),

                Math.atan2(getVectorComponent(velocity[0][2], acceleration[1][2], Component.Y),
                        getVectorComponent(velocity[0][2], acceleration[1][2], Component.X)),

                Math.atan2(getVectorComponent(velocity[0][3], acceleration[1][3], Component.Y),
                        getVectorComponent(velocity[0][3], acceleration[1][3], Component.X))
        );
    }

    private double getVectorComponent(double magnitude, double angle, Component type) {
        switch (type) {
            case X: { return magnitude * Math.cos(angle); }
            case Y: { return magnitude * Math.sin(angle); }
            default: { return 0; }
        }
    }

    protected void setLockWheelsPosition(LockedWheelPositions desiredPosition) { lockedStatus = desiredPosition; }

    protected void setLocked(boolean locked) { this.locked = locked; }

    protected boolean isLocked() { return locked; }
}
