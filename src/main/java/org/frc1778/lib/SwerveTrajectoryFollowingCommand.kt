package org.frc1778.lib

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.lib.PathPlannerTrajectoryStopEventBuilder.stopEventGroup
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential

class SwerveTrajectoryFollowingCommand(
    private val drivetrain: FalconSwerveDrivetrain<*>,
    private val trajectories: List<PathPlannerTrajectory>,
    private val eventMap: HashMap<String, Command>
) : FalconCommand(drivetrain) {
    private val command = sequential {
        +InstantCommand({
            drivetrain.resetPosition(trajectories.first().initialHolonomicPose)
        })
        for (trajectory in trajectories) {
            +stopEventGroup(trajectory.startStopEvent, eventMap)
            +drivetrain.followTrajectoryWithCommands(trajectory, eventMap)
        }
        +stopEventGroup(trajectories.last().endStopEvent, eventMap)
    }

    override fun initialize() {
        command.initialize()
    }

    override fun execute() {
        command.execute()
    }


    override fun cancel() {
        super.cancel()
        command.cancel()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }


}
