package org.firstinspires.ftc.teamcode.hardware.Util.SensorEx;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.VALUE_REJECTION;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CleverEncoder {
    private static double range = 3.3; //volts (V)

    private final AnalogInput encoder;
    private double offset = 0;
    private boolean reversed = false;

    private double voltage;
    private double position, lastPosition;

    public CleverEncoder(HardwareMap hardwareMap, String encoder_name) { this(hardwareMap, encoder_name, range); }

    public CleverEncoder(HardwareMap hardwareMap, String encoder_name, double actualRange) {
        encoder = hardwareMap.get(AnalogInput.class, encoder_name);
        this.range = actualRange;
    }

    public CleverEncoder zero() {
        setOffset(getPosition());
        return this;
    }

    public void reversed(boolean reversed) { this.reversed = reversed; }

    public boolean isReversed() { return reversed; }

    public double getPosition() {
        position = Angle.norm((reversed ? getVoltage() / range : 1 - getPosition() / range) * 2 * Math.PI - offset);

        if(!VALUE_REJECTION || Math.abs(Angle.normDelta(lastPosition)) > 0.1 || Math.abs(Angle.normDelta(position)) < 1)
            lastPosition = position;

        return lastPosition;
    }

    public void setOffset(double offset) { this.offset = offset; }

    public AnalogInput getEncoder() { return encoder; }

    public double getVoltage() { return voltage; }

    public void read() { voltage = encoder.getVoltage(); }
}
