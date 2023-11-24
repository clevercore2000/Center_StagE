package org.firstinspires.ftc.teamcode.Generals;

import org.firstinspires.ftc.teamcode.Unnamed.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.Unnamed.MotionProfiling.ProfileConstrains;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
