package org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathFollowers;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.headingP;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.xyP;

import org.firstinspires.ftc.teamcode.hardware.Generals.Localizer;
import org.firstinspires.ftc.teamcode.hardware.Generals.MotionProfile;
import org.firstinspires.ftc.teamcode.hardware.Generals.Paths;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.motion.WayFinder.MotionProfiling.ProfileConstrains;
import org.firstinspires.ftc.teamcode.motion.WayFinder.MotionProfiling.Trapezoidal.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathBuilders.BezierCurve;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Pathing.PathBuilders.PurePursuit;

import java.util.List;

public class GenericFollower {
    private List<Paths> pathsToFollow;
    private Localizer localizer;
    private Paths currentBuildPath;
    private Paths currentFollowedPath;
    private MotionProfile profile;

    private boolean maintainHeading = false;
    private boolean isStarted;
    private boolean isPaused;
    private boolean isBusy;

    Pose currentRobotPose;

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower(Localizer localizer) {
        this.localizer = localizer;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower addPath() {
        currentBuildPath.build();
        pathsToFollow.add(currentBuildPath);

        return this;
    }

    public GenericFollower addPoint(Point newPoint) {
        if (currentBuildPath != null)
            currentBuildPath.addPoint(newPoint);

        return this;
    }

    public GenericFollower build() {
        if (pathsToFollow.size() != 0)
            currentFollowedPath = pathsToFollow.get(0);
        else currentFollowedPath = null;

        currentBuildPath = null;
        isStarted = isPaused = isBusy = false;

        return this;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower newPurePursuit() {
        if (!(currentBuildPath instanceof PurePursuit)) { //if isn't a PurePursuit object => you forgot to add last type of Path lmao
            currentBuildPath.build();
            pathsToFollow.add(currentBuildPath);
            currentBuildPath = null;
        }

        if (currentBuildPath == null)
            currentBuildPath = new PurePursuit();


        return this;
    }

    public GenericFollower newBezierCurve() {
        if (!(currentBuildPath instanceof BezierCurve)) { //if isn't a PurePursuit object => you forgot to add last type of Path lmao
            currentBuildPath.build();
            pathsToFollow.add(currentBuildPath);
            currentBuildPath = null;
        }

        if (currentBuildPath == null)
            currentBuildPath = new BezierCurve(new Pose());


        return this;
    }

    public GenericFollower newTrapezoidalProfile(double start, double end, ProfileConstrains constrains) {
        profile = new TrapezoidalMotionProfile(start, end, constrains);

        return this;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void start() {
        isStarted = true;
    }

    public void pause() {
        if (currentFollowedPath != null)
            currentFollowedPath.pause();
        isPaused = true;
    }

    public void resume() {
        if (currentFollowedPath != null)
            currentFollowedPath.resume();
        isPaused = false;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public MotionSignal generateSignal() throws NotAPolynomialException {
        currentFollowedPath.update();

        Point currentFollowedPoint = currentFollowedPath.getPointToFollow();
        double heading = 0;

        if (!maintainHeading) { //you have 2 points and you want to face the target point, so you find the angle between the 2 points, easy
            heading = Math.atan2(currentFollowedPoint.y - currentRobotPose.y,
                    currentRobotPose.x - currentRobotPose.y);
        } else { heading = currentRobotPose.heading; } //just maintain heading, cuz why not

        MotionSignal signal = new MotionSignal();
        signal.velocity = new Pose(currentFollowedPoint.subtract(currentRobotPose.getPoint()).multiplyBy(xyP), heading * headingP);

        return signal;
    }

    //call this every loop
    public void read(){
        if (!currentFollowedPath.isBusy() && isStarted && !isPaused) {
            currentFollowedPath = pathsToFollow.iterator().hasNext() ? pathsToFollow.iterator().next() : null;
            if (currentFollowedPath != null)
                currentFollowedPath.start();
        }

        localizer.read();
        localizer.update();
        currentRobotPose = localizer.getRobotPosition();

        if (currentFollowedPath instanceof PurePursuit)
            ((PurePursuit) currentFollowedPath).setRobotPosition(currentRobotPose);

    }

    public boolean isBusy() {
        return currentFollowedPath != null && !isPaused;
    }

    public void maintainHeading(boolean maintainHeading) { this.maintainHeading = maintainHeading; }
}
