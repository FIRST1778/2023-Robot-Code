package org.frc1778.commands

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand

class ManipulatorClose : FalconCommand(Manipulator) {
    override fun initialize() {
        if(Manipulator.manipulatorOpen) {
                Manipulator.close()
        }
}

    override fun isFinished(): Boolean {
        return true
    }
}