package org.frc1778.commands

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

class ManipulatorOpenCommand : FalconCommand(Manipulator) {
    override fun initialize() {
        if(!Manipulator.manipulatorOpen || Manipulator.manipulatorSol.state == FalconSolenoid.State.Off){
            Manipulator.open()
        }
    }

    override fun isFinished(): Boolean {
        return true
    }
}