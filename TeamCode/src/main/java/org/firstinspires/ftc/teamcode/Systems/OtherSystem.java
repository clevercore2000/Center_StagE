package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.Drone;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.PullUp;

public class OtherSystem implements Enums{
    private static Drone drone;
    private static PullUp pullUp;

    private OtherSystem instance = null;

    private LinearOpMode opMode;

    public OtherSystem getInstance(LinearOpMode opMode) {
        if (instance == null)
            instance = new OtherSystem(opMode);
        return instance;
    }

    public OtherSystem(LinearOpMode opMode) {
        this.opMode = opMode;

        drone = new Drone(opMode.hardwareMap);
        pullUp = new PullUp(opMode.hardwareMap);
    }

    public void setState(PullUpPositions position) { pullUp.setState(position); }

    public void setState(LauncherPositions position) { drone.setState(position); }

    public void update() {
        drone.update();
        pullUp.update();
    }
}
