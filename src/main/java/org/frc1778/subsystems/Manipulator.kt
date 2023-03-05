package org.frc1778.subsystems

import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.frc1778.commands.ManipulatorCloseCommand
import org.frc1778.commands.ManipulatorOpenCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Manipulator : FalconSubsystem() {
    val manipulatorSol = FalconDoubleSolenoid(
            14,
            15,
            PneumaticsModuleType.REVPH,
            30
    )

    fun toggleState() {
        manipulatorSol.state = when(manipulatorSol.state) {
            FalconSolenoid.State.Forward -> FalconSolenoid.State.Reverse
            else -> FalconSolenoid.State.Forward
        }
    }

    fun open() {
        manipulatorSol.state = FalconSolenoid.State.Forward
    }

    fun close() {
        manipulatorSol.state = FalconSolenoid.State.Reverse
    }

}