package org.frc1778.commands

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand

class IntakeSuckCommand : FalconCommand(Intake) {
    override fun execute() {
        Intake.suck()
    }
}