package org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
