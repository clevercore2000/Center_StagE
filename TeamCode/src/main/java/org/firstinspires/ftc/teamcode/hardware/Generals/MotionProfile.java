package org.firstinspires.ftc.teamcode.hardware.Generals;

import org.firstinspires.ftc.teamcode.motion.WayFinder.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.motion.WayFinder.MotionProfiling.ProfileConstrains;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
