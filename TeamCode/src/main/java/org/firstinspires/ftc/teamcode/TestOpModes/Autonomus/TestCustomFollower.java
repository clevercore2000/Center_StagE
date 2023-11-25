package org.firstinspires.ftc.teamcode.TestOpModes.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Swerve.CleverSwerve;
import org.firstinspires.ftc.teamcode.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathFollowers.GenericFollower;

@Autonomous(group = "test", name = "WayFinderTest")
public class TestCustomFollower extends LinearOpMode {
    private CleverSwerve swerve;
    private GenericFollower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = swerve.getInstance(this, Enums.Swerve.Localizers.CUSTOM, Enums.OpMode.AUTONOMUS);
        swerve.setPoseEstimate(new Pose());

        follower = new GenericFollower(swerve.getLocalizer())
                .constructPurePursuit(new Point(20, 50))
                .constructPurePursuit(new Point( 100, 20))
                .constructPurePursuit(new Point(0, 0))
                .add()
                .build();

        waitForStart();

        follower.follow();

        while (opModeIsActive()) {
            swerve.read();
            follower.read();

            if (follower.isBusy()) {
                try { swerve.setMotionSignal(follower.generateSignal()); }
                    catch (NotAPolynomialException e) {}
            }

            swerve.update();
        }
    }
}
