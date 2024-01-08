package org.firstinspires.ftc.teamcode.hardware.Robot.Systems;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.allowOtherSystemUsageBeforeEndgame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.CleverBot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other.Drone;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.Subsystems.Other.PullUp;

public class OtherSystem implements Enums.Other {
    private Drone drone;
    private PullUp pullup;
    private CleverBot robotInstance;

    private ElapsedTime timerKnowingIfItIsEndgame;
    private final double timeNeedsToPassToReachEndgame = 120000; //ms

    private LinearOpMode opMode;


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public OtherSystem getInstance() { return this; }


        /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public OtherSystem(LinearOpMode opMode) {
        this.opMode = opMode;

        drone = new Drone(opMode.hardwareMap);
        pullup = new PullUp(opMode.hardwareMap);

        timerKnowingIfItIsEndgame = new ElapsedTime();
        timerKnowingIfItIsEndgame.startTime();
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public void setState(DronePosition position) { if (isAccessible()) drone.setState(position); }

    public void setRobotInstance(CleverBot robotInstance) { this.robotInstance = robotInstance; }

    public void setPower(double power) { if (isAccessible()) pullup.setPower(power); }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public void update() { }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */

    private boolean isEndgame() { return timerKnowingIfItIsEndgame.milliseconds() > timeNeedsToPassToReachEndgame; }

    private boolean isAccessible() { return isEndgame() || allowOtherSystemUsageBeforeEndgame; }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public void start() { timerKnowingIfItIsEndgame.reset(); }

    public void read() {}
}
