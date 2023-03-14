package org.frc1778.commands

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPoint
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

class BalanceDriveCommand : FalconCommand(Drive) {
    companion object {
        private const val MARGIN: Double = 0.2 // m
        private const val VELOCITY: Double = 0.5 // m/sec
        private const val FIELD_LENGTH: Double = 16.54 // m
        private const val OUT_OF_COMMUNITY_X: Double = 5.0 // m
        private const val BALANCE_BOARD_X: Double = 3.0
    }

    private var aprilTagsWereEnabled = false
    private lateinit var driveCommand: SwerveTrajectoryTrackerCommand

    private fun adjustForAlliance(t: Translation2d): Translation2d {
        var x: Double = t.x
        if (Robot.alliance == DriverStation.Alliance.Red) {
            x = FIELD_LENGTH - x
        }
        return Translation2d(x, t.y)
    }

    private fun makeCommand(): SwerveTrajectoryTrackerCommand {
        val currChassisSpeeds = Drive.kinematics.toChassisSpeeds(*Drive.swerveModuleStates().toTypedArray())
        val outOfCommunityPose = Pose2d(adjustForAlliance(Translation2d(OUT_OF_COMMUNITY_X, 0.0)), Drive.robotPosition.rotation)
        val balanceBoardPose = Pose2d(adjustForAlliance(Translation2d(BALANCE_BOARD_X, 0.0)), Rotation2d(0.0))
        val trajectory = PathPlanner.generatePath(
            PathConstraints(Drive.maxSpeed.value, 3.0),
            PathPoint(
                // field-relative position
                outOfCommunityPose.translation,
                // heading (direction robot travels in)
                Drive.robotPosition.rotation,
                // holonomic rotation (direction robot is facing)
                Drive.robotPosition.rotation
            ),
            PathPoint(
                balanceBoardPose.translation,
                Drive.robotPosition.rotation,
                0.0
            )
        )
        return Drive.followTrajectory(trajectory)
    }

    override fun initialize() {
        aprilTagsWereEnabled = Drive.aprilTagsEnabled
        Drive.aprilTagsEnabled = false
        driveCommand = makeCommand()
        driveCommand.schedule()
    }

    override fun cancel() {
        driveCommand.cancel()
    }

    override fun isFinished(): Boolean {
        return driveCommand.isFinished()
    }

    override fun end(interrupted: Boolean) {
        Drive.aprilTagsEnabled = aprilTagsWereEnabled
    }
}