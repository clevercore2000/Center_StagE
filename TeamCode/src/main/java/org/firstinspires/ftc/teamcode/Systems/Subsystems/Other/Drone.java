package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Generals.Enums;

public class Drone implements Enums.Other {
    private Servo servo;

    private final double load = 0.42, fire =  0.82;
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
