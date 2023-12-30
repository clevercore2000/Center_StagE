package org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.SwerveModule;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

import java.util.ArrayList;
import java.util.List;

public class SwerveState implements Enums.Swerve {


    public double speed;
    public double angle;
    private List<SwerveState> baseStates = new ArrayList<>();

    //default constructor
    public SwerveState() {}

    private SwerveState(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    public SwerveState(List<SwerveState> baseStates) {
        this.baseStates = baseStates;
    }

    public SwerveState add(double speed, double angle) {
        baseStates.add(new SwerveState(speed, angle));

        return this;
    }

    public double get(int i, state value) {
        if (i < baseStates.size() && i >= 0) {
            switch (value) {
                case SPEED: { return baseStates.get(i).speed; }
                case ANGLE: { return baseStates.get(i).angle; }
            }
        }

        return 0; //non-existent value
    }

    public SwerveState sum(SwerveState otherState) {
        if (this.getList().size() != otherState.getList().size())
            return this;
        else {
            SwerveState sum = new SwerveState();

            for(int i = 0; i <= this.getList().size(); i++) {
                sum.add(this.get(i, state.SPEED) + otherState.get(i, state.SPEED), this.get(i, state.ANGLE));
            }
            return sum;
        }
    }

    public SwerveState sum(double value) {
        for (SwerveState state : this.getList()) {
            state.speed += value;
        }
        return this;
    }

    public SwerveState multiplyBy(double value) {
        for (SwerveState state : this.getList()) {
            state.speed *= value;
        }
        return this;
    }

    public List<SwerveState> getList() { return baseStates; }
}
