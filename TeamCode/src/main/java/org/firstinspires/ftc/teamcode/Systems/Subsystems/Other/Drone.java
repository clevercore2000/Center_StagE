package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Systems.Enums;

class Drone {
    private CRServo tensioner;
    private Servo launcher;
    private Encoder encoder;

    private final double p = 0.01;
    private PIDController tensionController;

    private final double load = 0, fire = 0;
    private double targetTension = 0; // encoder ticks
    private HardwareMap hardwareMap;

    private Enums.LauncherPositions position;

    public Drone(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        tensioner = hardwareMap.get(CRServo.class, "tensioner");
        launcher = hardwareMap.get(Servo.class, "launcher");
        tensionController = new PIDController(p, 0, 0);

        //TODO: select the right swerve module port for this one
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));

        init();
    }


    /**initialization*/
    private void init() { setState(Enums.LauncherPositions.LOAD); }


    /**setter methods*/
    public void setTargetTension(double targetTension) { this.targetTension = targetTension; }

    public void setState(Enums.LauncherPositions position) {
        this.position =  position;

        switch (position) {
            case FIRE: { launcher.setPosition(fire); } break;
            case LOAD: { launcher.setPosition(load); } break;
            default: {}
        }
    }


    /**getter methods*/
    public double getTargetTension() { return targetTension; }


    /**update*/
    public void update() {
        double position = encoder.getCurrentPosition();
        double power = tensionController.calculate(position, targetTension);

        if (power > 0) { tensioner.setDirection(DcMotorSimple.Direction.FORWARD); }
        else { tensioner.setDirection(DcMotorSimple.Direction.REVERSE); }

        tensioner.setPower(power);
    }
    //TODO: find a way to use encoder ticks to use servos
}
