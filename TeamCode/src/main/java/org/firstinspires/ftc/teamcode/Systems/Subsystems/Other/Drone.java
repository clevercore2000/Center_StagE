package org.firstinspires.ftc.teamcode.Systems.Subsystems.Other;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Generals.Enums;

public class Drone implements Enums{
    private CRServo tensioner;
    private CRServo launcher;
    private Encoder encoder;

    private final double p = 0.01;
    private PIDController tensionController;

    private final double loadPower = 0.5, firePower = -0.5;
    private double targetTension = -4741; // encoder ticks
    private HardwareMap hardwareMap;

    private LauncherPositions position;

    public Drone(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        tensioner = hardwareMap.get(CRServo.class, "tensioner");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        tensionController = new PIDController(p, 0, 0);

        //TODO: select the right swerve module port for this one

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));

        init();
    }


    /**initialization*/
    private void init() { setState(LauncherPositions.LOAD); }


    /**setter methods*/
    public void setTargetTension(double targetTension) { this.targetTension = targetTension; }

    public void setState(LauncherPositions position) { this.position =  position; }

    public void setPower(double power) { tensioner.setPower(power); }


    /**getter methods*/
    public double getTargetTension() { return targetTension; }

    public double getEncoderPosition() { return encoder.getCurrentPosition(); }


    /**update*/
    public void update() {
        //double position = encoder.getCurrentPosition();
        //double power = tensionController.calculate(position, targetTension);

        //if (power > 0) { tensioner.setDirection(DcMotorSimple.Direction.FORWARD); }
        //else { tensioner.setDirection(DcMotorSimple.Direction.REVERSE); }

        switch (this.position) {
            case FIRE: { launcher.setPower(firePower); } break;
            case LOAD: { launcher.setPower(loadPower); } break;
            case PENDING: { launcher.setPower(0); } break;
            default: {}
        }

        //tensioner.setPower(power);
    }
    //TODO: find a way to use encoder ticks to use servos
}
