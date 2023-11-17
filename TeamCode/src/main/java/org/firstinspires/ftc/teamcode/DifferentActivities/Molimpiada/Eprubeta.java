package org.firstinspires.ftc.teamcode.DifferentActivities.Molimpiada;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
public class Eprubeta {
    private Servo servo;
    private ElapsedTime timerDoingNothing;

    private double UP = 0, POURING = 0.55;
    private double sensitivity = 0.01;
    private double timeDoingNoting = 100; //ms

    public Eprubeta(HardwareMap hardwareMap, String deviceName) {
      servo = hardwareMap.get(Servo.class, deviceName);
      timerDoingNothing = new ElapsedTime();

      timerDoingNothing.startTime();
    }

    public void goUp(){
        if (timerDoingNothing.milliseconds() >= timeDoingNoting && servo.getPosition() != UP) {
            servo.setPosition(servo.getPosition() - sensitivity);

            timerDoingNothing.reset();
        }
    }

    public void goPouring(){
        if (timerDoingNothing.milliseconds() >= timeDoingNoting && servo.getPosition() != POURING) {
            servo.setPosition(servo.getPosition() + sensitivity);

            timerDoingNothing.reset();
        }
    }

    public void Init(){
        servo.setPosition(UP);
    }

    public void reverseServo() { servo.setDirection(Servo.Direction.REVERSE); }

}
