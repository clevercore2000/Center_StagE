package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Enums;

@Deprecated
class Claw {
    private final Servo claw;

    private HardwareMap hardwareMap = null;

    private Enums.ClawPositions position;
    private final double OPEN = 0, CLOSED = 0.31;
    //TODO: find actual values

    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        claw = hardwareMap.get(Servo.class, "claw");

        init();
    }

    public void init() { setState(Enums.ClawPositions.OPEN); }

    public void setState(Enums.ClawPositions position) { this.position = position; }

    public void update() {
        switch (position) {
            case OPEN: { claw.setPosition(OPEN); } break;
            case CLOSED: { claw.setPosition(CLOSED); } break;
            default: {}
        }
    }


}
