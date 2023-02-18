package org.frc1778.subsystems

import edu.wpi.first.wpilibj.PneumaticsModuleType
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

class Manipulator : FalconSubsystem() {
    val manipulatorSol = FalconDoubleSolenoid(
            2,
            3,
            PneumaticsModuleType.REVPH,
            31
    )

    fun open(){
        manipulatorSol.state = FalconSolenoid.State.Forward
    }
    fun close(){
        manipulatorSol.state = FalconSolenoid.State.Reverse
    }
}