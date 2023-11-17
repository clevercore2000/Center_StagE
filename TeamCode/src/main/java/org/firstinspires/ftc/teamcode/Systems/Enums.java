package org.firstinspires.ftc.teamcode.Systems;

public class Enums {
    public enum Pixels{
        WHITE,
        YELLOW,
        GREEN,
        PURPLE,
        UNIDENTIFIED,
        NONE
    }

    public enum IntakeMotorStates{
        COLLECT,
        SPIT,
        STOP
    }

    public enum IntakeArmStates{
        UP,
        STACK_UP,
        STACK_DOWN,
        DOWN
    }

    public enum StoredPixels{
        ZERO,
        ONE,
        TWO,
        TOO_MANY
    }

    public enum OuttakeGripperStates{
        OPEN,
        CLOSED
    }

    public enum OuttakeRotationStates{
        COLLECT,
        SCORE
    }

    public enum LiftStates{
        COLLECT,
        INTERMEDIARY,
        LOW,
        MID,
        HIGH,
        UNDEFINED
    }

    public enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    public enum Sensors{
        FOR_PIXEL_1,
        FOR_PIXEL_2
    }

    public enum Timers{
        TIMER_FOR_SPITTING
    }

    public enum Pipelines{
        DETECTING_PIXELS_ON_BACKDROP
    }

    public enum Scoring{
        HIGH,
        MID,
        LOW
    }

    public enum Position{
        INTERMEDIARY,
        COLLECT
    }

    public enum PixelActions{
        GRAB,
        SCORE
    }

    public enum LauncherPositions{
        LOAD,
        FIRE
    }

    public enum ClawPositions{
        OPEN,
        CLOSED
    }

    public enum PullUpPositions{
        DOWN,
        UP,
        HANGING
    }
}

