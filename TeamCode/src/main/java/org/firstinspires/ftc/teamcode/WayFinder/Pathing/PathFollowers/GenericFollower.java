package org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathFollowers;

import static org.firstinspires.ftc.teamcode.Generals.SwerveConstants.headingP;
import static org.firstinspires.ftc.teamcode.Generals.SwerveConstants.xyP;

import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.Generals.MotionProfile;
import org.firstinspires.ftc.teamcode.Generals.Paths;
import org.firstinspires.ftc.teamcode.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.ProfileConstrains;
import org.firstinspires.ftc.teamcode.WayFinder.MotionProfiling.Trapezoidal.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathBuilders.PurePursuit;

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


    public GenericFollower(Localizer localizer) {
        this.localizer = localizer;
    }

    private GenericFollower(Localizer localizer, Paths currentBuildPath, List<Paths> pathsToFollow, MotionProfile profile) {
        this.localizer = localizer;
        this.pathsToFollow = pathsToFollow;
        this.currentBuildPath = currentBuildPath;
    }

    public GenericFollower add() {
        pathsToFollow.add(currentBuildPath);
        return new GenericFollower(localizer, null, pathsToFollow, profile);
    }

    public GenericFollower constructPurePursuit(Point pathPoint) {
        if (!(currentBuildPath instanceof PurePursuit)) { //if isn't a PurePursuit object => you forgot to add last type of Path lmao
            pathsToFollow.add(currentBuildPath);
            currentBuildPath = null;
        }

        if (currentBuildPath == null)
            currentBuildPath = new PurePursuit(localizer);

        ((PurePursuit) currentBuildPath).addPoint(pathPoint);

        return new GenericFollower(localizer, currentBuildPath, pathsToFollow, profile);
    }

    public GenericFollower constructTrapezoidalProfile(double start, double end, ProfileConstrains constrains) {
        profile = new TrapezoidalMotionProfile(start, end, constrains);

        return new GenericFollower(localizer, currentBuildPath, pathsToFollow, profile);
    }

    public GenericFollower build() {
        if (pathsToFollow.size() != 0)
            currentFollowedPath = pathsToFollow.get(0);
        else currentFollowedPath = null;

        currentBuildPath = null;
        isStarted = isPaused = isBusy = false;

        return this;
    }

    public MotionSignal generateSignal() throws NotAPolynomialException {
        currentFollowedPath.update();

        Point currentFollowedPoint = currentFollowedPath.getPointToFollow();
        Pose currentRobotPose = currentFollowedPath.getPosition();
        double heading = 0;

        if (!maintainHeading) { //you have 2 points and you want to face the target point, so you find the angle between the 2 points, easy
            heading = Math.atan2(currentFollowedPoint.y - currentRobotPose.y,
                    currentRobotPose.x - currentRobotPose.y);
        } else { heading = currentRobotPose.heading; } //just maintain heading, cuz why not

        MotionSignal signal = new MotionSignal();
        signal.velocity = new Pose(currentFollowedPoint.subtract(currentRobotPose.getPoint()).multiplyBy(xyP), heading * headingP);

        return signal;
    }

    public void follow() {
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

    public void read(){
        if (!currentFollowedPath.isBusy() && isStarted && !isPaused) {
            currentFollowedPath = pathsToFollow.iterator().hasNext() ? pathsToFollow.iterator().next() : null;
            if (currentFollowedPath != null)
                currentFollowedPath.start();
        }

        if (currentFollowedPath != null)
            currentFollowedPath.read();
    }

    public boolean isBusy() {
        return currentFollowedPath != null && !isPaused;
    }

    public void maintainHeading(boolean maintainHeading) { this.maintainHeading = maintainHeading; }
}
