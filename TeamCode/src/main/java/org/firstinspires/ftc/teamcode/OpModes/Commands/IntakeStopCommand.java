package org.firstinspires.ftc.teamcode.OpModes.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

public class IntakeStopCommand extends InstantCommand {

    public IntakeStopCommand(ScoringSystem scoring) {
            super( () -> scoring.stopIntake() );
        }

}

