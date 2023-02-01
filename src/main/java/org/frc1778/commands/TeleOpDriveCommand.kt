package org.frc1778.commands

import org.frc1778.Constants
import org.frc1778.Controls
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.Source
import kotlin.math.pow
import kotlin.math.withSign

class TeleOpDriveCommand : FalconCommand(Drive) {

    override fun execute() {
        Drive.swerveDrive(
            Controls.handleDeadBand(translationX(), 0.1).pow(2).withSign(
                translationX()
            ) * Constants.DriveConstants.maxSpeed.value,
            -Controls.handleDeadBand(translationY(), 0.1).pow(2).withSign(
                translationY()
            ) * Constants.DriveConstants.maxSpeed.value,
            -Controls.handleDeadBand(rotation(), 0.1).pow(2)
                .withSign(rotation()) * Constants.DriveConstants.maxAngularSpeed.value,
            true
        )

//        println("X: ${Controls.handleDeadBand(translationX(), 0.25)}")
//        println("Y: ${Controls.handleDeadBand(translationY(), 0.25)}")
//        println("Rot: ${Controls.handleDeadBand(rotation(), 0.25)}")

    }

    companion object {
        val translationX = Controls.driverController.getRawAxis(1)
        val translationY = Controls.driverController.getRawAxis(0)
//        val translationY: Source<Double> = {0.0}
        val rotation = Controls.driverController.getRawAxis(2)

    }
}