package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Util.MotionHardware.Init;

public class SwerveModule {
    private DcMotorEx speed;
    private Servo angle;

    private final double maxServoRangeInRadians = 300 * Math.PI / 180;
    private final double angleGearRatio = 2 / 1; //output / input (teeth)

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

    private void setSpeed(double power) {
        currentVelocity = (power > 0.02) ? (clipSpeed(power * flipModifier())) : 0;
        speed.setPower(currentVelocity);
    }

    private void setAngle(double a) {
        currentAngle = clipAngle(transformAngleIntoMotorInput(effectiveAngle(a))) ;
        angle.setPosition(currentAngle);

    }

    /**Finding shortest way to the desired position */
    private double effectiveAngle(double a) {

        if (MOTOR_FLIPPING && (a < 0 || a == PI)) {
            a = normalizeRadians((a == PI) ? a : a + PI) ;
            wheelFlipped = true;
        } else wheelFlipped = false;

        return a;
    }

    private double transformAngleIntoMotorInput(double a) { return a / maxServoRangeInRadians * angleGearRatio; }

    private double clipSpeed(double value) {
        if (value > 1)
            return 1;
        if (value < -1)
            return -1;

        return value;
    }


    private double clipAngle(double value) {
        if (value > 1)
            return  1;
        if (value < 0)
            return 0;

        return  value;
    }

    protected void run(double s, double a) {
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

    public double getActualCurrentAngle() { return currentAngle; }

    public double getVelocityError() { return Math.abs(getActualCurrentVelocity() - currentVelocity); }

    public double getAngleError() { return Math.abs(getActualCurrentAngle() - currentAngle); }

    public boolean isConstrained() { return Init.isConstrained(speed); }

    private int flipModifier() { return wheelFlipped ? -1 : 1; }

    public double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }
}
