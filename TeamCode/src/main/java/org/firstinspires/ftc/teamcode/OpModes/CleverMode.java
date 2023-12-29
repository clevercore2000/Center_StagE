package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class CleverMode extends LinearOpMode {
    protected abstract void Init();

    protected void WaitForStart() { super.waitForStart(); }

    protected abstract void WhenStarted();

    protected abstract void InitializeThreads();

    protected void AutonomusTasks() { idle(); }
}
