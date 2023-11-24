package org.firstinspires.ftc.teamcode.Unnamed.Pathing;

import static com.acmerobotics.roadrunner.util.MathUtil.solveQuadratic;
import static org.firstinspires.ftc.teamcode.Unnamed.Math.MathFormulas.QuadraticSolve;
import static org.firstinspires.ftc.teamcode.Unnamed.Math.MathFormulas.findLinearFunction;
import static org.firstinspires.ftc.teamcode.Unnamed.Math.MathFormulas.toPower;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;

import org.checkerframework.dataflow.qual.Pure;
import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.Unnamed.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Coefficients;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Point;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Pose;

import java.util.ArrayList;
import java.util.List;

public class PurePursuit {
    private double radiusToSearch = 20; //cm
    //TODO: tune this

    private int currentLineFirstPoint = 0;

    private Localizer robotLocalizer;
    private Pose robotPosition;
    private double circleCenterX, circleCenterY;

    private double pointsLeft;
    private List<Point> pathPoints = new ArrayList<Point>();
    private Point lastFollowedPoint = new Point();
    private Point pointToFollow;
    private Coefficients currentLineCoefficients;
    private Coefficients nextLineCoefficients;

    private boolean isOnSameLine = true;
    private boolean isStarted = false, isPaused = false, isBusy= false, isDone = false;
    private boolean useLastPointCorrection = true; //shifting the last point some small value from actual pose and use PID to get to final position

    private double lastThreePointsDifference = 0;
    private double correction = 100; //cm (applies to x and y)
    //TODO: tune this too
    private Point correctionPoint;

    public PurePursuit(Localizer localizer) { this.robotLocalizer = localizer; }

    public PurePursuit(Localizer localizer, double radius) {
        this.robotLocalizer = localizer;
        this.radiusToSearch = radius;
    }

    public PurePursuit(Localizer localizer, double radius, List<Point> pastPathPoints) {
        this.robotLocalizer = localizer;
        this.radiusToSearch = radius;
        this.pathPoints = pastPathPoints;
    }

    public PurePursuit addPoint(Point newPoint) {
        pathPoints.add(newPoint);
        return new PurePursuit(robotLocalizer, radiusToSearch, pathPoints);
    }

    public void start() {
        if (useLastPointCorrection) {
            lastThreePointsDifference = 1;
            addCorrectionLastPoint();
        }
        isStarted = true;
    }

    public void pause() { isPaused = true; }

    public void resume() { isPaused = false; }

    /**Hierarchy for following points:
     *      -points found on next line
     *      -points find on the current line
     *      -point closer to the end of the line (in case of multiple solutions for the same line)
     *      (+ never going back - checking with last point followed)
     *
     *
     * */
    public void update() throws NotAPolynomialException {
        updatePosition();

        if (isStarted && isBusy && !isPaused) {
            //because we are checking 3 consecutive points in one loop
            pointsLeft = pathPoints.size() - (currentLineFirstPoint + 3);
            //TODO: check how many points you still have in the list (special case for 2 and 1)


            //has 3 or more points left (can check 2 lines)
            if (pointsLeft >= lastThreePointsDifference) {

                //check if you find an intersection with the next line
                List<Point> intersectionsNextLine = findCircleIntersections(
                        pathPoints.get(currentLineFirstPoint + 1),
                        pathPoints.get(currentLineFirstPoint + 2));

                //if finding intersections, move onto the next line
                if (intersectionsNextLine != null) {
                    currentLineFirstPoint++;
                    pointsLeft--;
                    isOnSameLine = false;
                } else { isOnSameLine = true; }
            }


            //has 2 points left (can still check for points)
            if(pointsLeft >= lastThreePointsDifference - 1) {

                //find all intersections on the preferred line
                List<Point> intersectionsCurrentLine = findCircleIntersections(
                        pathPoints.get(currentLineFirstPoint),
                        pathPoints.get(currentLineFirstPoint + 1));

                //if none intersections are found, follow last found point
                if (intersectionsCurrentLine == null) {
                    pointToFollow = lastFollowedPoint;
                } else {

                    //if one intersection is found
                    if (intersectionsCurrentLine.size() == 1) {

                        //if is on the same line, find the desired point (as in the hierarchy)
                        if (isOnSameLine) {
                            pointToFollow = findClosestPointToEndOfTheLine(pointToFollow,
                                    intersectionsCurrentLine.get(0), pathPoints.get(currentLineFirstPoint + 1));
                        }

                        //if switched on next line, just set the following point as the found point
                        else {
                            pointToFollow = intersectionsCurrentLine.get(0);
                        }

                        //if two intersections are found
                    } else {

                        //find the desired point (as in the hierarchy)
                        pointToFollow = findClosestPointToEndOfTheLine(
                                intersectionsCurrentLine.get(0), intersectionsCurrentLine.get(1),
                                pathPoints.get(currentLineFirstPoint));

                        if (!isOnSameLine) {
                            //do another check with the last followed point. We account if, for some reason, the found point make the robot go backwards
                            pointToFollow = findClosestPointToEndOfTheLine(
                                    pointToFollow, pathPoints.get(currentLineFirstPoint + 1),
                                    pathPoints.get(currentLineFirstPoint));
                        }
                    }
                }
            } else {
                isBusy = false;
                isDone = true;
            }

            lastFollowedPoint = pointToFollow;
        }

    }

    private void addCorrectionLastPoint() {
        double lastPointX = pathPoints.get(pathPoints.size() - 1).x;
        double lastPointY = pathPoints.get(pathPoints.size() - 1).y;

        pathPoints.remove(pathPoints.get(pathPoints.size() - 1));
        pathPoints.add(new Point(lastPointX - correction, lastPointY - correction));
    }

    private void updatePosition() {
        robotLocalizer.update();

        robotPosition = robotLocalizer.getRobotPosition();
        circleCenterX = robotPosition.x;
        circleCenterY = robotPosition.y;
    }

    public void usingCorrection(boolean using) { this.useLastPointCorrection = using; }

    private List<Point> findCircleIntersections(Point point1, Point point2) throws NotAPolynomialException {
        Coefficients lineEcuationCoefficients = findLinearFunction(point1, point2);

        double lineEquation_a = lineEcuationCoefficients.getCoefficient(0).doubleValue();
        double lineEquation_b = lineEcuationCoefficients.getCoefficient(1).doubleValue();

        double a = toPower(lineEquation_a, 2) + 1;
        double b = 2 * (lineEquation_a * (lineEquation_b - circleCenterY) - circleCenterX);
        double c = (lineEquation_b - circleCenterY + radiusToSearch) * (lineEquation_b - circleCenterY - radiusToSearch) + toPower(circleCenterX, 2);

        double[] solutions = QuadraticSolve(a, b, c);
        if (solutions == null) { return null; }

        List<Point> intersectingPoints = new ArrayList<>();
        for(int i = 0; i <= solutions.length; i++) {
            double pointX = solutions[i];
            double pointY = lineEquation_a * pointX + lineEquation_b;

            intersectingPoints.add(new Point(pointX, pointY));
        }

        return intersectingPoints;
    }

    private Point findClosestPointToEndOfTheLine(Point point1, Point point2, Point end) {
        double firstPointDistance = distanceBetweenTwoPoints(point1, end);
        double secondPointDistance = distanceBetweenTwoPoints(point2, end);

        if (firstPointDistance < secondPointDistance) return point1;
        return point2;
    }

    private double distanceBetweenTwoPoints(Point point1, Point point2) {
        return Math.hypot(point2.x - point1.x, point2.y - point1.y);
    }

    public Pose getPosition() { return robotPosition; }

    public Point getPointToFollow() { return pointToFollow; }

    public Point getLastFollowedPoint() { return lastFollowedPoint; }


}

