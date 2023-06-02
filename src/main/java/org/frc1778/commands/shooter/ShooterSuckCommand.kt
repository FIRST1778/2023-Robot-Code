package org.frc1778.commands.shooter

import org.frc1778.subsystems.shooter.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterSuckCommand : FalconCommand(Shooter) {

    override fun initialize() {
        Shooter.suck()
    }

    override fun cancel() {
        super.cancel()
        end(true)
    }


    override fun end(interrupted: Boolean) {
        Shooter.stopWheels()
    }


    override fun isFinished(): Boolean {
        return Shooter.cubeStored
    }
}