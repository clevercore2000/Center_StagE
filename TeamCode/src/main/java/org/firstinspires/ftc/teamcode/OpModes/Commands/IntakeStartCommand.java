package org.firstinspires.ftc.teamcode.OpModes.Commands;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SystemConstants.useManualEnable;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.Systems.ScoringSystem;

public class IntakeStartCommand extends InstantCommand {

    public IntakeStartCommand(ScoringSystem scoring, Enums.Scoring.IntakeArmStates armState) {
        super(() -> { if (scoring.isManualIntakeEnabled() || !useManualEnable) scoring.startIntake(armState); });
    }
}
