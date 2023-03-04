package org.frc1778.commands

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand


class IntakeSpitCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.retract()
        Intake.spit()
    }

    override fun cancel() {
        super.cancel()
        Intake.stop()
    }
}