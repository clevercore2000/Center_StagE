package org.firstinspires.ftc.teamcode.WayFinder.Pathing.PathBuilders;

import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Generals.Paths;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

import java.util.ArrayList;
import java.util.List;

public class BezierCurve implements Enums.Pathing, Paths {
    private List<Point> pathPoints = new ArrayList<>();
    private Point lastFollowedPoint = new Point();
    private Point pointToFollow;

    private double increment = 0.001; //ranges from 0 to 1
    private double t = 0;

    private boolean isStarted = false, isPaused = false, isBusy= false, isDone = false;

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public BezierCurve(Point startPoint) { pathPoints.add(startPoint); }

    public BezierCurve(Point startPoint, double increment) {
        this.increment = increment;
        pathPoints.add(startPoint);
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Override
    public BezierCurve addPoint(Point newPoint) {
        pathPoints.add(newPoint);
        return this;
    }

    @Override
    public void build() {}

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Override
    public void start() { isStarted = true; isBusy = true; }

    @Override
    public void pause() { isPaused = true; }

    @Override
    public void resume() { isPaused = false; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void update() {
        if (isStarted && isBusy && !isPaused && t <= 1) {
            lastFollowedPoint = pointToFollow;
            List<Point> interpolations = pathPoints;

            while (interpolations.size() > 1) {
                for (int i = 0; i < interpolations.size() - 1; i++) {
                    Point interpolatedPoint = linearInterpolation(interpolations.get(i), interpolations.get(i + 1), t);
                    interpolations.set(i, interpolatedPoint);
                }
                interpolations.remove(interpolations.size() - 1);
            }

            pointToFollow = interpolations.get(0);
            t += increment;
        } else if (t > 1){ isBusy = false; isDone = true; }
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    private Point linearInterpolation(Point point1, Point point2, double t) {
        double x = point1.x * (1 - t) + point2.x;
        double y = point1.y * (1 - t) + point2.y;

        return new Point(x, y);
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void setIncrement(double increment) { this.increment = increment; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Override
    public Point getPointToFollow() { return pointToFollow; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


    public boolean isStarted() { return isStarted; }

    public boolean isPaused() { return isPaused; }

    @Override
    public boolean isBusy() { return isBusy; }

    public boolean isDone() { return isDone; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
}
