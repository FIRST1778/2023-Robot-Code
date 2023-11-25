package org.frc1778.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.frc1778.Constants
import org.frc1778.Controls
import org.frc1778.Robot
import org.frc1778.subsystems.drive.Drive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import kotlin.math.pow
import kotlin.math.withSign

class TeleOpDriveCommand: FalconCommand(Drive) {
    private var autoAlignWasActive = false
    private lateinit var goalRotation: Rotation2d

//    private fun resetAutoAlign() {
//        val initialState = Gyro.odometryYaw()
//        val initialSpeeds = Drive.kinematics.toChassisSpeeds(
//            *Drive.swerveModuleStates().toTypedArray()
//        )

        // Round angle to 180.
//        val goalState = PI * round(initialState / PI)
//        goalRotation = Rotation2d(goalState)
        //PPHolonomicDriveController breaks this.
        // Could create another controller here using the same constants.
//        Drive.controller.thetaController.reset(
//            TrapezoidProfile.State(
//                initialState, initialSpeeds.omegaRadiansPerSecond
//            )
//        )
//    }

    override fun execute() {
        var radiansPerSecond: Double

//        if (autoAlignActive()) {
//            if (!autoAlignWasActive) {
//                resetAutoAlign()
//            }
//
//            radiansPerSecond = Drive.controller.thetaController.calculate(
//                Gyro.odometryYaw(), goalRotation.radians
//            )
//        } else {
            // Get rotation from the joystick.  Only use the up/down axis.
            radiansPerSecond = -Controls.handleDeadBand(rotation(), 0.1).pow(2)
                .withSign(rotation()) * Constants.DriveConstants.maxAngularSpeed.value
//        }
        Drive.swerveDrive(
            Controls.handleDeadBand(translationX(), 0.1).pow(2).withSign(
                translationX() * Robot.driveInversion
            ) * Constants.DriveConstants.maxSpeed.value,
            -Controls.handleDeadBand(translationY(), 0.1).pow(2).withSign(
                translationY() * Robot.driveInversion
            ) * Constants.DriveConstants.maxSpeed.value,
            radiansPerSecond, true
        )

        autoAlignWasActive = autoAlignActive()
    }

    companion object {
        val translationX = -Controls.driverController.getRawAxis(5)
        val translationY = Controls.driverController.getRawAxis(4)
        val rotation = Controls.driverController.getRawAxis(0)
//        val rotation = {.25}
        // There are two switches at the top of the controller.  Right is hold, with
        // ID 1; left is toggle, with ID 2.
        val autoAlignActive = Controls.driverController.getRawButton(1)

        private operator fun DoubleSource.unaryMinus(): DoubleSource {
            return { -this() }
        }
    }
}
