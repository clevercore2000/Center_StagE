package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.Drone;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.PullUp;

public class OtherSystem implements Enums.Other {
    private static Drone drone;
    private static PullUp pullUp;

    private ElapsedTime timerKnowingIfItIsEndgame;
    private final double timeNeedsToPassToReachEndgame = 120000; //ms
    private static OtherTimer usingTimer;

    private OtherSystem instance = null;

    private LinearOpMode opMode;

    public OtherSystem getInstance(LinearOpMode opMode, OtherTimer usingTimer) {
        this.usingTimer = usingTimer;
        instance = new OtherSystem(opMode);

        return instance;
    }

    public OtherSystem(LinearOpMode opMode) {
        this.opMode = opMode;

        drone = new Drone(opMode.hardwareMap);
        pullUp = new PullUp(opMode.hardwareMap);

        timerKnowingIfItIsEndgame = new ElapsedTime();
        timerKnowingIfItIsEndgame.startTime();
    }

    public void setState(PullUpPositions position) { if (isEndgame() || usingTimer == OtherTimer.NOT_USING_ENDGAME_TIMER) pullUp.setState(position); }

    public void setState(LauncherPositions position) { if (isEndgame() || usingTimer == OtherTimer.NOT_USING_ENDGAME_TIMER) drone.setState(position); }

    public void update() {
        if (isEndgame()) {
            drone.update();
            pullUp.update();
        }
    }

    private boolean isEndgame() { return usingTimer == OtherTimer.USING_ENDGAME_TIMER && timerKnowingIfItIsEndgame.milliseconds() > timeNeedsToPassToReachEndgame; }

    public void start() { timerKnowingIfItIsEndgame.reset(); }

    public void read() {
        pullUp.read();
        drone.read();
    }
}
