package org.firstinspires.ftc.teamcode.Swerve.SwerveModule;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Generals.Constants.SwerveConstants.feelingRisky;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Util.MotionHardware.Init;

public class SwerveModule {
    private DcMotorEx speed;
    private Servo angle;

    private final double maxServoRangeInRadians = Math.toRadians(1800);
    private final double angleGearRatio = 2 / 1; //output / input (teeth)
    private double offsetInRadians = 0;

    private final double PI = Math.PI;

    /**Distance units in CM*/
    public static double WHEEL_RADIUS = 26.5;
    public static double GEAR_RATIO = 1; //to be tuned
    public static double MAX_RPM = 312;
    public static double MOTOR_TICKS_PER_REV = 517.5;
    public static double WHEEL_TICKS_PER_REV = MOTOR_TICKS_PER_REV / GEAR_RATIO;

    public static double MAX_MOTOR = 1;

    public static boolean MOTOR_FLIPPING = true;

    private double currentVelocity = 0.0;
    private double currentAngle = 0.0;
    private double previousAngle = 0.0;
    private double currentAcceleration = 0.0;


    public boolean wheelFlipped = false;

    public void initialize() {
        /**Enhancing motor (steroids basically)*/
        MotorConfigurationType motorConfigurationType = speed.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        speed.setMotorType(motorConfigurationType);

        speed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speed.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**Enhancing servo (also steroids)*/
        ((ServoImplEx) angle).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public void setDirection(DcMotorSimple.Direction direction) { speed.setDirection(direction); }

    public void setDirection(Servo.Direction direction) { angle.setDirection(direction); }

    public SwerveModule(HardwareMap hardwareMap, String motor_name, String servo_name) {
        speed = hardwareMap.get(DcMotorEx.class, motor_name);
        angle = hardwareMap.get(Servo.class, servo_name);

        initialize();
    }

    public SwerveModule(DcMotorEx speed, Servo angle) {
        this.speed = speed;
        this.angle = angle;

        initialize();
    }

    public void setOffset(double offset) { offsetInRadians = offset; }

    public void fromServoPowerToAngle(double offset) {
        setOffset(offset * maxServoRangeInRadians);
    }

    private void setSpeed(double power) {
        currentVelocity = (power > 0.02) ? (clipSpeed(power * flipModifier())) : 0;
        speed.setPower(currentVelocity);
    }

    private void setAngle(double a) {
        previousAngle = currentAngle;
        currentAngle = clipAngle(transformAngleIntoServoInput(effectiveAngle(a))) ;
        angle.setPosition(currentAngle);

    }

    /**Finding shortest way to the desired position */
    private double effectiveAngle(double a) {
        /*if (feelingRisky) {
            double normalizedPreviousAngle = normalizeRadians(previousAngle);
            double error = normalizeRadians(a - normalizedPreviousAngle);

            if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
                previousAngle += Math.signum(error) * normalizeRadians(a - Math.PI);
                wheelFlipped = true;
            } else {
                previousAngle += Math.signum(error) * normalizeRadians(a);
                wheelFlipped = false;
            }

            return previousAngle;

        } else {*/
            if (MOTOR_FLIPPING && (a < 0 || a == PI)) {
                a = normalizeRadians((a == PI) ? a : a + PI);
                wheelFlipped = true;
            } else wheelFlipped = false;

            return a;
        //}

    }

    private double transformAngleIntoServoInput(double a) { return a / maxServoRangeInRadians * angleGearRatio; }

    private double clipSpeed(double value) { return Range.clip(value, -1, 1); }

    private double clipAngle(double value) { return Range.clip(value, 0, 1); }

    public void run(double s, double a) {
        setAngle(a);
        setSpeed(s);
    }

    public double getCurrentPosition() { return encoderTicksToCM(speed.getCurrentPosition()); }

    public void resetCurrentPosition() {
        Init.resetEncoders(speed);
    }

    public double getTargetVelocity() { return  currentVelocity; }

    public double getTargetAngle() { return currentAngle; }

    public double getActualCurrentVelocity() { return encoderTicksToCM(speed.getVelocity()); }

    public double getActualCurrentAngle() { return previousAngle; }

    public double getVelocityError() { return Math.abs(getActualCurrentVelocity() - currentVelocity); }

    public double getAngleError() { return currentAngle - previousAngle; }

    public boolean isConstrained() { return Init.isConstrained(speed, 4); }

    private int flipModifier() { return wheelFlipped ? -1 : 1; }

    public double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }
}
