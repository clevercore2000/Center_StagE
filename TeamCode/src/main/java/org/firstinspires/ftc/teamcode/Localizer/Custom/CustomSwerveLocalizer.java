package org.firstinspires.ftc.teamcode.Localizer.Custom;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Util.Math.Pose;

public class CustomSwerveLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0; //to be tuned (inch)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public double l = 0; //also inches
    public double r = 0;
    public double b = 0;

    private Pose currentPose, previousPose, velocityPose;

    private double previousL, previousR, previousB;
    private double currentL, currentR, currentB;
    private double delta_L, delta_R, delta_B;

    private ElapsedTime loopTime;

    private Encoder leftEncoder, rightEncoder, backEncoder;
    private Dead3WheelLocalizer localizer = null;

    public static double encoderTicksToInch(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public CustomSwerveLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();
        previousPose = new Pose();
        velocityPose = new Pose();
        loopTime = new ElapsedTime();
        previousL = previousR = previousB = 0;
        currentL = currentR = currentB = 0;

        localizer = new Dead3WheelLocalizer(l, r, b);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @Override
    public void update() {
        updateEncoders();

        Pose deltaPose = localizer.getDeltaPoseEstimate(delta_L, delta_R, delta_B, currentPose.heading);
        currentPose = new Pose(previousPose.x + deltaPose.x, previousPose.y + deltaPose.y, deltaPose.heading);

        velocityPose = deltaPose.dividedBy(loopTime.seconds());
    }

    public Pose getRobotPosition() { return currentPose; }

    /**UNIT: m/s*/
    public Pose getRobotVelocity() { return velocityPose; }

    public void setPositionEstimate(Pose newPose) {
        currentPose = newPose;
        localizer.setStartOrientation(newPose.heading);
    }

    @Override
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return currentPose.heading; }
            case DEGREES: { return fromRadiansToDegrees(currentPose.heading); }
            default: { return 0; }
        }
    }

    public void updateEncoders() {
        loopTime.reset();
        previousL = currentL;
        previousR = currentR;
        previousB = currentB;

        currentL = leftEncoder.getCurrentPosition();
        currentR = rightEncoder.getCurrentPosition();
        currentB = backEncoder.getCurrentPosition();

        delta_L = encoderTicksToInch(currentL - previousL);
        delta_R = encoderTicksToInch(currentR - previousR);
        delta_B = encoderTicksToInch(currentB - previousB);

        previousPose = currentPose;
    }

}
