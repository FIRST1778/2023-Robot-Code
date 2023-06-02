package org.frc1778.commands.drive

import org.frc1778.subsystems.drive.Drive
import org.ghrobotics.lib.commands.FalconCommand

class DriveBrakeCommand: FalconCommand(Drive) {
    override fun initialize() {
        Drive.swerveDrive(0.0, 0.0, .125)
    }

    override fun end(interrupted: Boolean) {
        Drive.swerveDrive(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return true
    }
}