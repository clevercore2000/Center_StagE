package org.firstinspires.ftc.teamcode.Localizer.Custom;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.WayFinder.Localization.Pose;

public class CustomSwerveLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0; //cm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public double l = 0; //also cm
    public double r = 0;
    public double b = 0;

    private Pose currentPose, previousPose, velocityPose;

    private double previousL, previousR, previousB;
    private double currentL, currentR, currentB;
    private double delta_L, delta_R, delta_B;

    private ElapsedTime loopTime;

    private Encoder leftEncoder, rightEncoder, backEncoder;
    private Dead3WheelLocalizer localizer;

    public static double encoderTicksToCM(double ticks) {
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

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //TODO: select the right swerve modules for this
    }

    @Override
    public void update() {

        Pose deltaPose = localizer.getDeltaPoseEstimate(delta_L, delta_R, delta_B, currentPose.heading);
        currentPose = new Pose(currentPose.getPoint().sum(deltaPose.getPoint()), deltaPose.heading);

        velocityPose = deltaPose.divideBy(loopTime.seconds());
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

    /**call this every loop*/
    public void read() {
        loopTime.reset();
        previousL = currentL;
        previousR = currentR;
        previousB = currentB;

        currentL = leftEncoder.getCurrentPosition();
        currentR = rightEncoder.getCurrentPosition();
        currentB = backEncoder.getCurrentPosition();

        delta_L = encoderTicksToCM(currentL - previousL);
        delta_R = encoderTicksToCM(currentR - previousR);
        delta_B = encoderTicksToCM(currentB - previousB);

        previousPose = currentPose;
    }

}
