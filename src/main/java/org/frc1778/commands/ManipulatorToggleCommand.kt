package org.frc1778.commands

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

class ManipulatorToggleCommand : FalconCommand(Manipulator){
    override fun initialize() {
        Manipulator.manipulatorInitial = Manipulator.manipulatorOpen
    }

    override fun execute() {
        if(Manipulator.manipulatorSol.state != FalconSolenoid.State.Off && Manipulator.manipulatorOpen == Manipulator.manipulatorInitial){
            if(Manipulator.manipulatorOpen) Manipulator.close() else Manipulator.open()
        }
    }

    override fun isFinished(): Boolean {
        return (Manipulator.manipulatorSol.state == FalconSolenoid.State.Off) || (!Manipulator.manipulatorInitial)
    }
}
