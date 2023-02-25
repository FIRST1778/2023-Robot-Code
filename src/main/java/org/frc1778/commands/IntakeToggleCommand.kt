package org.frc1778.commands

import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.wrappers.FalconSolenoid

class IntakeToggleCommand(private val desiredPosition : FalconSolenoid.State? = null)  : WaitCommand(.5){
    init{
        addRequirements(Intake)
    }
    private fun setDesiredPos(initialState : FalconSolenoid.State) : FalconSolenoid.State{
        return desiredPosition ?: if(initialState == FalconSolenoid.State.Forward) FalconSolenoid.State.Reverse else FalconSolenoid.State.Forward
    }
    override fun initialize() {
        val initialState = Intake.intakeSol.state
        val desiredPos : FalconSolenoid.State = setDesiredPos(initialState)
        if(desiredPos != FalconSolenoid.State.Forward) Intake.extend() else Intake.retract()
    }
}