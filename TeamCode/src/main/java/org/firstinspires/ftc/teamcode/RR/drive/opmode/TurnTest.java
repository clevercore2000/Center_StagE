package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "RR_Turn", group = "RR")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    CleverSwerve swerve;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.CUSTOM, Enums.OpMode.AUTONOMUS);

        waitForStart();

        if (isStopRequested()) return;

        swerve.turnAsync(Math.toRadians(ANGLE));

        while (swerve.isBusy() && opModeIsActive() && !isStopRequested()) {}
    }
}