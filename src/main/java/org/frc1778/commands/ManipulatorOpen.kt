package org.frc1778.commands

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand

class ManipulatorOpen : FalconCommand(Manipulator) {
    override fun initialize() {
        Manipulator.open()
    }

    override fun isFinished(): Boolean {
        return true
    }
}