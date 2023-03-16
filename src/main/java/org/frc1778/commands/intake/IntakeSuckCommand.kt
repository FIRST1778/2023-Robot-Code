package org.frc1778.commands.intake

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand

class IntakeSuckCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.extend()
        Intake.suck()
    }

    override fun cancel() {
        Intake.stop()
        Intake.retract()
        super.cancel()
    }
}