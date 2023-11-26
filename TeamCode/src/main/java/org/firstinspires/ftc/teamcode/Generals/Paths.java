package org.firstinspires.ftc.teamcode.Generals;

import org.firstinspires.ftc.teamcode.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

public interface Paths {
    void update() throws NotAPolynomialException;

    Paths addPoint(Point newPoint);
    void build();

    void start();
    void pause();
    void resume();

    boolean isBusy();

    Point getPointToFollow();

}
