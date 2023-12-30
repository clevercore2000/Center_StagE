package org.firstinspires.ftc.teamcode.OpModes.Commands;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;


public class OuttakeScoreCommand extends InstantCommand {

    public OuttakeScoreCommand (ScoringSystem scoring, Enums.Scoring.Score scoringOption)
    {
        super(() -> { if (scoring.isManualOuttakeEnabled() || !useManualEnable) scoring.score(scoringOption); } );
    }
}
