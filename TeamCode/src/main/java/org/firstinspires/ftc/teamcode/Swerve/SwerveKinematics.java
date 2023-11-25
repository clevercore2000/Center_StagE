package org.firstinspires.ftc.teamcode.Swerve;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule.SwerveState;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

public abstract class SwerveKinematics implements Enums.Swerve {

    /**Robot-specific parameters*/
    public static double BASE_WIDTH = 13, BASE_LENGTH = 13; //to be tuned
    private final double drivetrain_radius = Math.hypot(BASE_WIDTH, BASE_LENGTH) / 2;
    private final double ANGULAR_VELOCITY_X = BASE_LENGTH / drivetrain_radius;
    private final double ANGULAR_VELOCITY_Y = BASE_WIDTH / drivetrain_radius;

    protected boolean locked = false;
    private LockedWheelPositions lockedStatus = LockedWheelPositions.DIAMOND;

    private SwerveState lastState = new SwerveState()
            .add(0, 0)
            .add(0, 0)
            .add(0, 0)
            .add(0, 0);


    /**Performing inverse kinematics to determine each module's state from a drivetrain state*/
    public SwerveState robot2moduleVelocity(Pose p) {
        double x = p.x, y = p.y, heading = p.heading;
        SwerveState state;

        double  left = x - heading * ANGULAR_VELOCITY_X,
                right = x + heading * ANGULAR_VELOCITY_X,
                front = y + heading * ANGULAR_VELOCITY_Y,
                back = y - heading * ANGULAR_VELOCITY_Y;

        if (locked) { state = getLockedWheelsPosition(); }
        else
        {
            //here you set desired order of wheel states
            state = new SwerveState()
                    .add(Math.hypot(front, left), Math.atan2(front, left))
                    .add(Math.hypot(front, right), Math.atan2(front, right))
                    .add(Math.hypot(back, right), Math.atan2(back, right))
                    .add(Math.hypot(back, left), Math.atan2(back, left));
        }


        lastState = state;
        return normalizeSpeeds(lastState);
    }

    public SwerveState robot2moduleAcceleration(Pose p) {
        return robot2moduleVelocity(p);
    }

    /**Performing forward kinematics to determine drivetrain's state from module states*/
    public Pose module2robot(SwerveState state) {
        double x = 0, y = 0, heading;

        for (int i = 0; i < 4 ; i++) {
            x += (state.get(i, Enums.Swerve.state.SPEED) * Math.cos(state.get(i, Enums.Swerve.state.ANGLE)));
            y += state.get(i, Enums.Swerve.state.SPEED) * Math.sin(state.get(i, Enums.Swerve.state.ANGLE));
        }
        heading = Math.atan2(y, x);

        return new Pose(x, y, heading);
    }


    private SwerveState normalizeSpeeds(SwerveState state) {
        double max = 0;
        for (SwerveState eachState : state.getList()) { max = Math.max(eachState.speed, max); }

        if (max > 1)
        for (SwerveState eachState : state.getList()) { eachState.speed /= max; }

        return state;
    }


    private SwerveState getLockedWheelsPosition() {

        switch (lockedStatus) {
            case DIAMOND: {
                SwerveState state = new SwerveState();
                for (int i = 0; i < 4; i++)
                    state.add(0, ((i % 2 == 0) ? -1 : 1) * Math.PI / 4);

                return state;
            }

            case X_SHAPE: {
                SwerveState state = new SwerveState();
                for (int i = 0; i < 4; i++)
                    state.add(0, ((i % 2 == 0) ? 1 : -1) * Math.PI / 4);

                return state;
            }

            case DEFAULT: {
                SwerveState state = new SwerveState();
                for (int i = 0; i < 4; i++)
                    state.add(0, lastState.get(i, Enums.Swerve.state.ANGLE));

                return state;
            }

            default: {
                SwerveState state = new SwerveState();
                for (int i = 0; i < 4; i++)
                    state.add(0, 0);

                return state;
            }
        }

    }

    private double getVectorComponent(double magnitude, double angle, Component type) {
        switch (type) {
            case X: { return magnitude * Math.cos(angle); }
            case Y: { return magnitude * Math.sin(angle); }
            default: { return 0; }
        }
    }

    protected void setLockStatus(LockedWheelPositions lockedStatus) { this.lockedStatus = lockedStatus; }

    protected void setLocked(boolean locked) { this.locked = locked; }

    protected boolean isLocked() { return locked; }
}
