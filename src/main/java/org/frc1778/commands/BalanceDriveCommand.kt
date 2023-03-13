package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.geometry.Translation2d

class BalanceDriveCommand : FalconCommand(Drive) {
    companion object {
        private const val MARGIN: Double = 0.2 // m
        private const val VELOCITY: Double = 0.5 // m/sec
        private const val FIELD_LENGTH: Double = 16.54 // m
    }

    private var aprilTagsWereEnabled = false
    private var driveCommand

    private fun adjustForAlliance(t: Translation2D) {
        var x = t.x
        if (DriverStation.alliance == DriverStation.Alliance.Red) {
            x = FIELD_LENGTH - x
        }
        return Translation2D(x, t.y)
    }

    private fun makeCommand() {
        val currChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates().toTypedArray())
        val outOfCommunityPose = adjustForAlliance(Translation2D(5, 0))
        val balanceBoardPose = adjustForAlliance(Translation2D(3, 0))
        val trajectory = PathPlanner.generatePath(
            PathConstraints(Drive.maxSpeed.value, 3.0),
            PathPoint(
                // field-relative position
                outOfCommunityPose,
                // heading (direction robot travels in)
                Transform2d(Drive.robotPosition, outOfCommunityPose).translation.angle,
                // holonomic rotation (direction robot is facing)
                Drive.robotPosition.rotation
            ),
            PathPoint(
                balanceBoardPose,
                Transform2D(outOfCOmmunityPose, balanceBoardPose).translation.angle,
                0.0
            )
        )
        return Drive.followTrajectory(trajectory)
    }

    override fun initialize() {
        aprilTagsWereEnabled = Drive.aprilTagsEnabled
        Drive.aprilTagsEnabled = false
        driveCommand = makeCommand()
    }

    override fun execute() {
        
    }

    override fun isFinished(): Boolean {
        return stage == xs.size
    }

    override fun end(interrupted: Boolean) {
        Drive.aprilTagsEnabled = aprilTagsWereEnabled
    }
}