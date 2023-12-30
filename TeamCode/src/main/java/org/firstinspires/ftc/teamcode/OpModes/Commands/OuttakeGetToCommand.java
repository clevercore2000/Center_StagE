package org.firstinspires.ftc.teamcode.OpModes.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

public class OuttakeGetToCommand extends InstantCommand {

    public OuttakeGetToCommand(ScoringSystem scoring, Enums.Scoring.Position position) {
        super(() -> scoring.getTo(position));
    }
}
