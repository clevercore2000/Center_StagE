package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Enums;

public class OtherSystem {
    private static Drone drone;
    private static Claw claw;
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
        claw = new Claw(opMode.hardwareMap);
        pullUp = new PullUp(opMode.hardwareMap);
    }

    public void setState(Enums.PullUpPositions position) { pullUp.setState(position); }

    //public void setState(Enums.ClawPositions position) { claw.setState(position); }

    public void setState(Enums.LauncherPositions position) { drone.setState(position); }

    public void update() {
        claw.update();
        drone.update();
        pullUp.update();
    }
}
