package org.frc1778.commands

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand


class IntakeStopCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.retract()
        Intake.stop()
    }

    override fun isFinished(): Boolean {
        return true
    }
}