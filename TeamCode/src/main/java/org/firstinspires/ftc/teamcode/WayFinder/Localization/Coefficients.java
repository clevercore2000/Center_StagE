package org.firstinspires.ftc.teamcode.WayFinder.Localization;

import org.firstinspires.ftc.teamcode.WayFinder.Exceptions.NotAPolynomialException;

import java.util.ArrayList;
import java.util.List;

public class Coefficients extends Point{
    public enum Type{
        UNDEFINED, constant, linear, quadratic, cubic, quartic, quintic, MULTIPLE
    }

    private List<Double> coefficients = new ArrayList<>();
    private Type polynomialType = Type.UNDEFINED;

    public Coefficients (List<Double> desiredCoefficient) throws NotAPolynomialException {
        for (int i = 0; i < desiredCoefficient.size(); i++) {
            coefficients.add(desiredCoefficient.get(i));
        }

        if (coefficients.size() == 0) {
            throw new NotAPolynomialException("you can't pass no coefficients");
        } else {
            if (coefficients.size() == 1) { polynomialType = Type.constant; }
            else if (coefficients.size() == 2) { polynomialType = Type.linear; }
            else if (coefficients.size() == 3) { polynomialType = Type.quadratic; }
            else if (coefficients.size() == 4) { polynomialType = Type.cubic; }
            else if (coefficients.size() == 5) { polynomialType = Type.quartic; }
            else if (coefficients.size() == 6) { polynomialType = Type.quintic; }
            else polynomialType = Type.MULTIPLE;
        }

        if (coefficients.get(0) == 0) { throw new NotAPolynomialException("first coefficient can't be 0"); }
    }

    public Type getPolynomialType() { return polynomialType; }

    public Double getCoefficient(int i) { return coefficients.get(i); }
}
