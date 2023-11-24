package org.firstinspires.ftc.teamcode.Generals;

public interface Enums {
    enum Pixels{
        WHITE,
        YELLOW,
        GREEN,
        PURPLE,
        UNIDENTIFIED,
        NONE
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

    enum StoredPixels{
        ZERO,
        ONE,
        TWO
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

    enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    enum Sensors{
        FOR_PIXEL_1,
        FOR_PIXEL_2
    }

    enum Pipelines{
        DETECTING_PIXELS_ON_BACKDROP
    }

    enum Scoring{
        HIGH,
        MID,
        LOW
    }

    enum Position{
        INTERMEDIARY,
        COLLECT
    }

    enum PixelActions{
        GRAB,
        SCORE
    }

    enum LauncherPositions{
        LOAD,
        FIRE,
        PENDING
    }

    enum PullUpPositions{
        DOWN,
        UP,
        HANGING,
        PENDING
    }

    enum OpMode{
        TELE_OP,
        AUTONOMUS
    }

    enum AfterSpitting {
        GET_BACK,
        STOP
    }

    enum Update{
        NONE,
        INTAKE,
        OUTTAKE,
        SCORING_SYSTEM,
        SENSORS,
        ALL
    }
}

