package org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.SwerveModule;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.K_STATIC;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.MOTOR_FLIPPING;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.angleD;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.moduleP;
import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.usingAxons;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Util.MotionHardware.Init;
import org.firstinspires.ftc.teamcode.hardware.Util.SensorEx.CleverEncoder;

public class SwerveModule {
    private DcMotorEx speed;
    private Servo angle;

    private CleverEncoder encoder;
    private PIDController controller;

    private final double maxServoRangeInRadians = Math.toRadians(1800);
    private final double angleGearRatio = 2 / 1; //output / input (teeth)

    private final double PI = Math.PI;

    /**Distance units in CM*/
    public static double WHEEL_RADIUS = 26.5;
    public static double GEAR_RATIO = 1; //to be tuned
    public static double MAX_RPM = 312;
    public static double MOTOR_TICKS_PER_REV = 517.5;
    public static double WHEEL_TICKS_PER_REV = MOTOR_TICKS_PER_REV / GEAR_RATIO;

    public static double MAX_MOTOR = 1;

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

    public void setDirection(Servo.Direction direction) { ((Servo) angle).setDirection(direction); }

    public void setDirection(boolean reversed) { if (encoder != null) encoder.reversed(reversed); }

    public void setOffset(double offset) { if (encoder != null) encoder.setOffset(offset); }

    public void setPID(double p, double i, double d) { if (controller!= null) controller.setPID(p, i, d); }

    public SwerveModule(HardwareMap hardwareMap, String motor_name, String servo_name, String encoder_name) {
        speed = hardwareMap.get(DcMotorEx.class, motor_name);

        if (usingAxons) {
            //angle = hardwareMap.get(CRServo.class, servo_name);

            encoder = new CleverEncoder(hardwareMap, encoder_name).zero();
            controller = new PIDController(moduleP, 0, angleD);
        } else {
            angle = hardwareMap.get(Servo.class, servo_name);
        }

        initialize();
    }

    public SwerveModule(DcMotorEx speed, Servo angle, CleverEncoder encoder) {
        this.speed = speed;
        this.angle = angle;

        if (usingAxons)
            this.encoder = encoder;

        initialize();
    }

    public void fromServoPowerToAngle(double offset) {
        setOffset(offset * maxServoRangeInRadians);
    }

    private void setSpeed(double power) {
        currentVelocity = (power > 0.02) ? (clipSpeed(power * flipModifier())) : 0;
        speed.setPower(currentVelocity);
    }

    private void setAngle(double a) {
        previousAngle = currentAngle;

        if (!usingAxons) {
            currentAngle = clipAngle(transformAngleIntoServoInput(effectiveAngle(a)));
            angle.setPosition(currentAngle);
        } else {
            currentAngle = normalizeRadians(a);

            double target = getModuleTarget(), current = getModulePosition();
            double error = normalizeRadians(target - current);

            if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
                target = normalizeRadians(target - Math.PI);
                wheelFlipped = true;
            } else { wheelFlipped = false; }

            error = normalizeRadians(target - current);

            double power = clipSpeed(controller.calculate(0, error));
            if (Double.isNaN(power)) power = 0;
            ((CRServo) angle).setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
        }

    }

    /**Finding shortest way to the desired position */
    private double effectiveAngle(double a) {
        if (MOTOR_FLIPPING && (a < 0 || a == PI)) {
            a = normalizeRadians((a == PI) ? a : a + PI);
            wheelFlipped = true;
        } else wheelFlipped = false;

        return a;
    }

    private double transformAngleIntoServoInput(double a) { return a / maxServoRangeInRadians * angleGearRatio; }

    private double clipSpeed(double value) { return Range.clip(value, -1, 1); }

    private double clipAngle(double value) { return Range.clip(value, 0, 1); }

    public void run(double s, double a) {
        setAngle(a);
        setSpeed(s);
    }

    public double getCurrentPosition() { return encoderTicksToCM(speed.getCurrentPosition()); }

    public void resetCurrentPosition() { Init.resetEncoders(speed); }

    public double getTargetVelocity() { return  currentVelocity; }

    public double getModuleTarget() { return normalizeRadians(currentAngle - Math.PI); }

    public double getModulePosition() { return  encoder.getPosition(); }

    public double getActualCurrentVelocity() { return encoderTicksToCM(speed.getVelocity()); }

    public double getActualCurrentAngle() { return previousAngle; }

    public double getVelocityError() { return Math.abs(getActualCurrentVelocity() - currentVelocity); }

    public double getAngleError() { return currentAngle - previousAngle; }

    public boolean isConstrained() { return Init.isConstrained(speed, 4); }

    private int flipModifier() { return wheelFlipped ? -1 : 1; }

    public double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / WHEEL_TICKS_PER_REV;
    }

    public void read() { if (encoder != null) encoder.read(); }
}
