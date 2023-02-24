package org.frc1778.subsystems

import edu.wpi.first.wpilibj.PneumaticsModuleType
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Manipulator : FalconSubsystem() {
    val manipulatorSol = FalconDoubleSolenoid(
            2,
            3,
            PneumaticsModuleType.REVPH,
            31
    )
    var manipulatorOpen : Boolean = false
    var manipulatorInitial = false
    fun toggle(){
        if(manipulatorOpen){
            close()
        }
        else{
            open()
        }
    }
    fun open(){
        manipulatorSol.state = FalconSolenoid.State.Forward
        manipulatorOpen = true
    }
    fun close(){
        manipulatorSol.state = FalconSolenoid.State.Reverse
        manipulatorOpen = false
    }
}