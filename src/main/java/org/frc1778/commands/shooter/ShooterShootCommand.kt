package org.frc1778.commands.shooter

import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterShootCommand : FalconCommand(Shooter) {
    override fun initialize() {
        Shooter.cubeStored = false
        Shooter.shoot(Shooter.getScoringLevel().shooterVoltage)
    }

    override fun cancel() {
        super.cancel()
        Shooter.stopWheels()
    }
}