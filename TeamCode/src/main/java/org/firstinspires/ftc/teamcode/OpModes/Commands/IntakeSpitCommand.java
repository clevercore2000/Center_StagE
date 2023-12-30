package org.firstinspires.ftc.teamcode.OpModes.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

public class IntakeSpitCommand extends InstantCommand {

    public IntakeSpitCommand(ScoringSystem scoring) {
        super( () -> scoring.spitIntake() );
    }
}
