package org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

public class Drone implements Enums.Other {
    private Servo servo;

    private final double load = 0.5, fire =  0.0;
    private DronePosition position = DronePosition.LOAD;

    public Drone(HardwareMap hardwareMap) { servo = hardwareMap.get(Servo.class, "drone"); update(); }

    public void setState(DronePosition position) { this.position = position; update(); }

    public DronePosition getState() { return position; }

    private void update() {
        switch (position) {
            case LOAD: { servo.setPosition(load); } break;
            case FIRE: { servo.setPosition(fire); } break;

        }
    }
}
