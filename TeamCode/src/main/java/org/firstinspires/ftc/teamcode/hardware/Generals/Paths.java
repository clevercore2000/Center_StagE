package org.firstinspires.ftc.teamcode.hardware.Generals;

import org.firstinspires.ftc.teamcode.motion.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Point;

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
