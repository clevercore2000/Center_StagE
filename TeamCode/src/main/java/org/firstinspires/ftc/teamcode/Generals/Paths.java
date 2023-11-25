package org.firstinspires.ftc.teamcode.Generals;

import org.firstinspires.ftc.teamcode.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Point;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

public interface Paths {
    void update() throws NotAPolynomialException;

    void read();

    void start();
    void pause();
    void resume();

    boolean isBusy();

    Pose getPosition();

    Point getPointToFollow();

}
