package org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathFollowers;

import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;

import lombok.Data;

@Data public class MotionSignal {
    public Pose velocity;
    public Pose acceleration;
}
