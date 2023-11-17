package org.firstinspires.ftc.teamcode.Util.MotionHardware;

@Deprecated
public class Constrains {
    public double MAX_velocity;
    public double MAX_acceleration;
    public double MAX_angularVelocity;
    public double MAX_angularAcceleration;
    public double MAX_jerk;

    public Constrains (double v, double a, double av, double aa, double j) {
        MAX_velocity = v;
        MAX_acceleration = a;
        MAX_angularVelocity = av;
        MAX_angularAcceleration = aa;
        MAX_jerk = j;
    }

    public void scale(double factor) {
        MAX_velocity *= factor;
        MAX_acceleration *= factor;
        MAX_angularVelocity *= factor;
        MAX_angularAcceleration *= factor;
        MAX_jerk *= factor;
    }
}
