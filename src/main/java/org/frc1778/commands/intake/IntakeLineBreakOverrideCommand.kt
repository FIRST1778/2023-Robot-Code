package org.frc1778.commands.intake

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand

class IntakeLineBreakOverrideCommand : FalconCommand() {
    override fun initialize() {
        Intake.lineBreakOverrideToggle()
    }

    override fun isFinished(): Boolean {
         return true
    }
}