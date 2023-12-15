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
        swerve = new CleverSwerve(this, Enums.Swerve.Localizers.CUSTOM, Enums.OpMode.AUTONOMUS);
        swerve.setPoseEstimate(new Pose());

        follower = new GenericFollower(swerve.getLocalizer())
                .newPurePursuit()
                    .addPoint(new Point(20, 50))
                    .addPoint(new Point( 100, 20))
                    .addPoint(new Point(0, 0))
                    .addPath()
                .newBezierCurve()
                    .addPoint(new Point(40, 40))
                    .addPoint(new Point(12, 60))
                    .addPoint(new Point( 100, 100))
                    .addPath()
                .build();

        waitForStart();

        follower.start();

        while (opModeIsActive()) {
            follower.read();

            if (follower.isBusy()) {
                try { swerve.setMotionSignal(follower.generateSignal()); }
                    catch (NotAPolynomialException e) {}
            }

            swerve.update();
        }
    }
}
