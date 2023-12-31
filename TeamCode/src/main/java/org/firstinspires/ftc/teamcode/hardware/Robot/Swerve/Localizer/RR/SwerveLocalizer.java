package org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.RR;

import static org.firstinspires.ftc.teamcode.motion.WayFinder.Math.Transformations.Pose2d_2_Pose;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Localizer;
import org.firstinspires.ftc.teamcode.motion.WayFinder.Localization.Pose;
import org.firstinspires.ftc.teamcode.motion.RR.util.Encoder;

import java.util.Arrays;
import java.util.List;

/**Roadrunner implementation of 3 odometry wheel localizer*/
public class SwerveLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.57; //to be tuned (inch)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //to be tuned (inch) - because roadrunner, duhh
    public static double LEFT_DISTANCE = -6.3;
    public static double RIGHT_DISTANCE = 4.67;
    public static double LATERAL_DISTANCE = -16.84;
    public static double PERPENDICULAR_DISTANCE = 0.02;//2.75;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static double encoderTicksToInch(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public SwerveLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(PERPENDICULAR_DISTANCE, 0, Math.toRadians(90)) // front/back/perpendicular
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInch(leftEncoder.getCurrentPosition()),
                encoderTicksToInch(rightEncoder.getCurrentPosition()),
                encoderTicksToInch(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInch(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInch(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInch(frontEncoder.getCorrectedVelocity())
        );
    }

    public Pose getRobotPosition() { return Pose2d_2_Pose(super.getPoseEstimate()); }

    public Pose getRobotVelocity() { return Pose2d_2_Pose(super.getPoseVelocity()); }

    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return super.getPoseEstimate().getHeading(); }
            case DEGREES: { return fromRadiansToDegrees(super.getPoseEstimate().getHeading()); }
            default: { return 0; }
        }
    }



    public void setPositionEstimate(Pose newPose) {
        Pose2d pose2d = new Pose2d(newPose.x, newPose.y, newPose.heading);
        super.setPoseEstimate(pose2d);
    }



    @Override
    public void update() { super.update(); }

    public void read() {}
}
