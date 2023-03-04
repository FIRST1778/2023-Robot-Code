package org.frc1778.commands

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

class ManipulatorOpenCommand : FalconCommand(Manipulator) {
    override fun execute() {
        Manipulator.manipulatorSol.state = FalconSolenoid.State.Forward
    }


    override fun isFinished(): Boolean {
        return true
    }
}