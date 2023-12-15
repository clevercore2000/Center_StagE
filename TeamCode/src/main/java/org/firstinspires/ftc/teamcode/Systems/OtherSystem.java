package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants.allowOtherSystemUsageBeforeEndgame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.Drone;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.PullUp;

public class OtherSystem implements Enums.Other {
    private static Drone drone;
    private static PullUp pullUp;

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
        pullUp = new PullUp(opMode.hardwareMap);

        timerKnowingIfItIsEndgame = new ElapsedTime();
        timerKnowingIfItIsEndgame.startTime();
    }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public void setState(PullUpPositions position) { if (isAccessible()) pullUp.setState(position); }

    public void setState(DronePosition position) { if (isAccessible()) drone.setState(position); }


    /*
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     */


    public void update() { pullUp.update(); }


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

    public void read() { pullUp.read(); }
}
