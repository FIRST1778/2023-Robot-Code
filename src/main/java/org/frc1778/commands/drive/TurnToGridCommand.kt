package org.frc1778.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand

class TurnToGridCommand : FalconCommand(Drive) {

    private lateinit var targetRotation: Rotation2d

    override fun initialize() {
        targetRotation = Rotation2d.fromDegrees(
            when (Robot.alliance) {
                DriverStation.Alliance.Red -> 0.0
                else -> 180.0
            }
        )


        val initialState = Drive.robotPosition.rotation
        val initialSpeeds = Drive.kinematics.toChassisSpeeds(*Drive.swerveModuleStates().toTypedArray())

        Drive.controller.thetaController.reset(
            TrapezoidProfile.State(
                initialState.radians, initialSpeeds.omegaRadiansPerSecond
            )
        )

    }

    override fun execute() {
        val omegaRadiansPerSecond = Drive.controller.thetaController.calculate(
            Drive.robotPosition.rotation.radians, targetRotation.radians
        )

        val wheelStates = Drive.kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, omegaRadiansPerSecond))

        Drive.setOutputSI(wheelStates)
    }

    override fun end(interrupted: Boolean) {
        Drive.setNeutral()
    }

    override fun isFinished(): Boolean {
        return Drive.controller.thetaController.atGoal()
    }

}