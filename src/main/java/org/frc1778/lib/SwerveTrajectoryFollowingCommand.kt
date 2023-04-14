package org.frc1778.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.lib.PathPlannerTrajectoryStopEventBuilder.stopEventGroup
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.subsystems.Drive
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
        drivetrain.controller.thetaController.reset(drivetrain.robotPosition.rotation.radians)
        command.initialize()
    }

    override fun execute() {
        command.execute()
    }


    override fun end(interrupted: Boolean) {
        command.end(interrupted)
    }

    override fun cancel() {
        super.cancel()
        command.cancel()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }


}
