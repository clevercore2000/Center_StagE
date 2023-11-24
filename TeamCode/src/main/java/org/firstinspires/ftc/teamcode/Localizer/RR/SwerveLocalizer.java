package org.firstinspires.ftc.teamcode.Localizer.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Generals.Localizer;
import org.firstinspires.ftc.teamcode.Unnamed.Localization.Pose;
import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Unnamed.Math.Transformations;

import java.util.Arrays;
import java.util.List;

/**Roadrunner implementation of 3 odometry wheel localizer*/
public class SwerveLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {
    private final Transformations t = new Transformations();

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0; //to be tuned (inch)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //to be tuned (inch) - because roadrunner, duhh
    public static double left_parallel_x = 0;
    public static double right_parallel_x = 0;
    public static double perpendicular_x = 0;

    public static double left_parallel_y = 0;
    public static double right_parallel_y = 0;
    public static double perpendicular_y = 0;


    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static double encoderTicksToInch(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public SwerveLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(left_parallel_x, left_parallel_y, 0), // left
                new Pose2d(right_parallel_x, right_parallel_y, 0), // right
                new Pose2d(perpendicular_x, perpendicular_y, Math.toRadians(90)) // front/back/perpendicular
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));

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

    public Pose getRobotPosition() { return t.Pose2d_2_Pose(super.getPoseEstimate()); }

    public Pose getRobotVelocity() { return t.Pose2d_2_Pose(super.getPoseVelocity()); }

    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return super.getPoseVelocity().getHeading(); }
            case DEGREES: { return fromRadiansToDegrees(super.getPoseVelocity().getHeading()); }
            default: { return 0; }
        }
    }



    public void setPositionEstimate(Pose newPose) {
        Pose2d pose2d = new Pose2d(newPose.x, newPose.y, newPose.heading);
        super.setPoseEstimate(pose2d);
    }



    @Override
    public void update() { super.update(); }
}
