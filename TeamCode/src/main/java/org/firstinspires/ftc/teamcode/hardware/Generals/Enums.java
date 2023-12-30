package org.firstinspires.ftc.teamcode.hardware.Generals;

public interface Enums {


    enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    enum Pipelines{
        DETECTING_PIXELS_ON_BACKDROP,
        DETECTING_PROP
    }

    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }

    enum OpMode{
        TELE_OP,
        AUTONOMUS
    }

    enum Gamepads{
        G1, G2,
        BOTH
    }


    interface Scoring {
        enum Pixels{
            WHITE,
            YELLOW,
            GREEN,
            PURPLE,
            UNIDENTIFIED,
            NONE
        }

        enum StoredPixels{
            ZERO,
            ONE,
            TWO
        }

        enum PixelActions{
            GRAB,
            SCORE,
            COLLECT_GRAB;
        }

        enum IntakeMotorStates{
            COLLECT,
            SPIT,
            STOP
        }

        enum IntakeArmStates{
            UP,
            STACK_UP,
            STACK_DOWN,
            DOWN
        }

        enum AfterSpitting {
            GET_BACK,
            STOP
        }

        enum OuttakeGripperStates{
            OPEN,
            CLOSED
        }

        enum OuttakeRotationStates{
            COLLECT,
            SCORE
        }

        enum LiftStates{
            COLLECT,
            INTERMEDIARY,
            LOW,
            MID,
            HIGH,
            UNDEFINED
        }

        enum Score{
            HIGH,
            MID,
            LOW
        }

        enum Position{
            INTERMEDIARY,
            COLLECT
        }

        enum Update{
            NONE,
            INTAKE,
            OUTTAKE,
            SCORING_SYSTEM,
            SENSORS,
            ALL
        }

        enum Sensors{
            FOR_PIXEL_1,
            FOR_PIXEL_2
        }
    }



    interface Other {
        enum DronePosition {
            LOAD,
            FIRE
        }

        enum PullUpPositions{
            DOWN,
            UP,
            HANGING,
            PENDING
        }
    }

    interface Swerve {
        enum MotionPackage {
            ROADRUNNER,
            CUSTOM
        }

        enum Localizers {
            CUSTOM,
            IMU,
            ROADRUNNER
        }

        enum ModuleOffset {
            POSITION,
            ANGLE
        }

        enum state{
            SPEED,
            ANGLE
        }

        enum LockedWheelPositions{
            DIAMOND,
            X_SHAPE,
            DEFAULT
        }

        enum Component {
            X,
            Y
        }
    }

    interface Pathing{
        enum Polynomial{
            UNDEFINED, constant, linear, quadratic, cubic, quartic, quintic, MULTIPLE
        }
    }
}

