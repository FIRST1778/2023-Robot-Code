package org.frc1778.commands

import edu.wpi.first.math.geometry.Pose2d
import org.frc1778.Robot
import org.frc1778.commands.SwerveTrajectoryTrackerCommand
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand

class PlaceGameObjectDriveCommand : FalconCommand(Drive) {
    private var scoringTrajectoryTrackerCommand: SwerveTrajectoryTrackerCommand? = null
    
    override fun initialize() {
        scoringTrajectoryTrackerCommand = Drive.trajectoryToGoal()?.let { Drive.followTrajectory(it) }
        scoringTrajectoryTrackerCommand?.schedule()
    }

    override fun cancel() {
        super.cancel()
        scoringTrajectoryTrackerCommand?.cancel()
    }

    override fun end(interrupted: Boolean) {
        if(!interrupted) {
            Robot.placeGameObjectCommand.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return scoringTrajectoryTrackerCommand?.isFinished() ?: false
    }
}