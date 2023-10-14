package org.frc1778.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import org.frc1778.Constants
import org.frc1778.Controls
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Gyro
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.abs
import kotlin.math.PI
import kotlin.math.round
import kotlin.math.withSign

class TeleOpDriveCommand: FalconCommand(Drive) {
    private fun deadband(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }

    private fun scale(x: Double): Double = (x*x).withSign(x)

    private var autoAlignWasActive = false
    private lateinit var goalRotation: Rotation2d

    private fun resetAutoAlign() {
        val initialState = Gyro.odometryYaw()
        val initialSpeeds = Drive.kinematics.toChassisSpeeds(
            *Drive.swerveModuleStates().toTypedArray()
        )

        // Round angle to 180.
        val goalState = PI * round(initialState / PI)
        goalRotation = Rotation2d(goalState)

        Drive.controller.thetaController.reset(
            TrapezoidProfile.State(
                initialState, initialSpeeds.omegaRadiansPerSecond
            )
        )
    }

    override fun execute() {
        var radiansPerSecond: Double
        val state = Controls.driveState()

        if (state.align) {
            if (!autoAlignWasActive) {
                resetAutoAlign()
            }

            radiansPerSecond = Drive.controller.thetaController.calculate(
                Gyro.odometryYaw(), goalRotation.radians
            )
        } else {
            // Get rotation from the joystick.  Only use the up/down axis.
            radiansPerSecond = -scale(deadband(state.rot, 0.1)) * Constants.DriveConstants.maxAngularSpeed.value
        }

        val modifier = Robot.driveInversion * Constants.DriveConstants.maxSpeed.value

        Drive.swerveDrive(
            scale(deadband(state.dx, 0.1)) * modifier,
            scale(deadband(state.dy, 0.1)) * modifier,
            radiansPerSecond, true
        )

        autoAlignWasActive = state.align
    }
}
