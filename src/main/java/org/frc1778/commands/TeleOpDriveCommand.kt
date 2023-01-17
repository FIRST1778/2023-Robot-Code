package org.frc1778.commands

import org.frc1778.Controls
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand

class TeleOpDriveCommand : FalconCommand(Drive){

    override fun execute() {
        Drive.swerveDrive(translationX(), translationY(), rotation(), true)
    }

    companion object {
        val translationX = Controls.driverController.getRawAxis(0)
        val translationY = Controls.driverController.getRawAxis(0)
        val rotation = Controls.driverController.getRawAxis(0)

    }
}