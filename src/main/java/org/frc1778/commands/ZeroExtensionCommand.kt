package org.frc1778.commands

import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand

class ZeroExtensionCommand : FalconCommand(Arm){
    override fun initialize(){
        Arm.resetIsZeroed()
    }
    override fun execute(){
        Arm.doExtensionZeroingMovement()
    }
    override fun isFinished(): Boolean {
        return Arm.limitSwitchHit()
    }

    override fun end(interrupted: Boolean) {
        Arm.zeroExtension()
    }
}