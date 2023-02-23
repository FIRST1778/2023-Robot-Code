package org.frc1778.commands

import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.meters

class ZeroExtensionCommand : FalconCommand(Arm){
    override fun initialize(){

    }
    override fun execute(){
        Arm.extensionMotor.setVoltage(-0.5)
    }
    override fun isFinished(): Boolean {
        return Arm.limitSwitch.get()
    }

    override fun end(interrupted: Boolean) {
        Arm.extensionEncoder.reset()
        Arm.extensionMotor.setVoltage(0.0)
        Arm.zeroed = true
        Arm.desiredExtension = 0.0.meters
    }
}