package org.frc1778.commands.intake

import org.frc1778.subsystems.intake.Intake
import org.ghrobotics.lib.commands.FalconCommand

class IntakeLowerCommand : FalconCommand(Intake) {

    override fun initialize() {
        Intake.stop()
        Intake.extend()
    }

    override fun cancel() {
        super.cancel()
        Intake.retract()
    }
}