package org.frc1778.commands.shooter

import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterStopCommand : FalconCommand() {
    override fun initialize() {
        Shooter.stopWheels()
    }

    override fun isFinished(): Boolean {
        return true
    }
}