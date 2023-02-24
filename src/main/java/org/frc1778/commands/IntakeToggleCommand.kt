package org.frc1778.commands

import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

class IntakeToggleCommand(val desiredPosition : FalconSolenoid.State =  if(Intake.initialState == FalconSolenoid.State.Forward) FalconSolenoid.State.Reverse else FalconSolenoid.State.Forward) : WaitCommand(.5){
    init{
        addRequirements(Intake)
    }

    override fun initialize() {
        if(desiredPosition != FalconSolenoid.State.Forward) Intake.extend() else Intake.retract()

    }
}