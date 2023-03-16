package org.frc1778.commands.intake

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand


class IntakeSpitCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.spit()
        Intake.retract()
    }

    override fun cancel() {
        Intake.stop()
        super.cancel()
    }
}