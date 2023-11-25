package org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathFollowers;

import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

import lombok.Data;

@Data public class MotionSignal {
    public Pose velocity;
    public Pose acceleration;
}
