package org.frc1778.commands

import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

class ToggleIntakeCommand : FalconCommand(Intake){
    override fun initialize() {
        Intake.initialState = Intake.intakeSol.state
    }

    override fun execute() {
        if(Intake.intakeSol.state != FalconSolenoid.State.Off){
            if(Intake.initialState == FalconSolenoid.State.Forward) Intake.retract() else Intake.extend()
        }
    }

    override fun isFinished(): Boolean {
        return (Intake.intakeSol.state == FalconSolenoid.State.Off) || (Intake.intakeSol.state != Intake.initialState)
    }
}