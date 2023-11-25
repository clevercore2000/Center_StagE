package org.firstinspires.ftc.teamcode.Generals;

import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.ProfileConstrains;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
