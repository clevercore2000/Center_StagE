package org.firstinspires.ftc.teamcode.Util.Math.MotionProfiling;

import androidx.annotation.NonNull;

public class MotionState {
    public enum Stage {
        T1, T2, T3, T4, T5, T6, T7
    }

    public enum val {
        POSITION,
        VELOCITY,
        ACCELERATION,
        JERK,
        TIME
    }


    private double current_position;
    private double current_velocity;
    private double current_acceleration;
    private double current_jerk;

    private Stage current_stage;
    private double current_time_in_stage;

    public MotionState(double p, double v, double a, double j, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = a;
        this.current_jerk = j;

        this.current_stage = stage;
        this.current_time_in_stage = t;
    }

    public MotionState(double p, double v, double a, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = a;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_stage = t;
    }

    public MotionState(double p, double v, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_stage = t;
    }

    public MotionState(double v, Stage stage, double t) {
        this.current_position = 0;
        this.current_velocity = v;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_stage = t;
    }

    public MotionState(Stage stage, double t) {
        this.current_position = 0;
        this.current_velocity = 0;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_stage = t;
    }


    public double get(val value) {
        switch (value) {
            case POSITION: { return current_position; }
            case VELOCITY: { return current_velocity; }
            case ACCELERATION: { return current_acceleration; }
            case JERK: { return current_jerk; }
            case TIME: { return current_time_in_stage; }
            default: { return 0; }
        }
    }

    public Stage getStage() { return current_stage; }

}
