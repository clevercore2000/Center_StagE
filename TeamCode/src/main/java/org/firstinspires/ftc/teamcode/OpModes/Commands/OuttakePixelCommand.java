package org.firstinspires.ftc.teamcode.OpModes.Commands;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

public class OuttakePixelCommand extends InstantCommand {

    public OuttakePixelCommand(ScoringSystem scoring, Enums.Scoring.PixelActions action) {
        super(() -> { if (scoring.isManualOuttakeEnabled() || !useManualEnable) scoring.pixels(action, false); });
    }
}
