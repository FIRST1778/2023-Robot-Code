package org.frc1778.lib

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential

class SwerveTrajectoryGroupFollowingCommand(
    private val drivetrain: FalconSwerveDrivetrain<*>,
    private val trajectories: List<PathPlannerTrajectory>,
    private val eventMap: HashMap<String, Command>
): FalconCommand(drivetrain) {
    private val command = sequential {
        for(trajectory in trajectories) {
            +stopEventGroup(trajectory.startStopEvent)
            +drivetrain.followTrajectoryWithCommands(trajectory, eventMap)
        }
        +stopEventGroup(trajectories.last().endStopEvent)
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

    private fun stopEventGroup(stopEvent: PathPlannerTrajectory.StopEvent): CommandBase {
        if (stopEvent.names.isEmpty()) {
            return Commands.waitSeconds(stopEvent.waitTime)
        }

        val eventCommands: CommandBase = stopEventCommands(stopEvent)

        return when (stopEvent.waitBehavior) {
            PathPlannerTrajectory.StopEvent.WaitBehavior.BEFORE -> Commands.sequence(Commands.waitSeconds(stopEvent.waitTime), eventCommands)
            PathPlannerTrajectory.StopEvent.WaitBehavior.AFTER -> Commands.sequence(eventCommands, Commands.waitSeconds(stopEvent.waitTime))
            PathPlannerTrajectory.StopEvent.WaitBehavior.DEADLINE -> Commands.deadline(Commands.waitSeconds(stopEvent.waitTime), eventCommands)
            PathPlannerTrajectory.StopEvent.WaitBehavior.MINIMUM -> Commands.parallel(Commands.waitSeconds(stopEvent.waitTime), eventCommands)
            PathPlannerTrajectory.StopEvent.WaitBehavior.NONE -> eventCommands
            else -> eventCommands
        }
    }

    @OptIn(ExperimentalStdlibApi::class)
    private fun stopEventCommands(stopEvent: PathPlannerTrajectory.StopEvent): CommandBase {
        val commands = mutableListOf<CommandBase>()
        val startIndex = if (stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE) 1 else 0
        for (i in startIndex..<stopEvent.names.size) {
            val name = stopEvent.names[i]
            if (eventMap.containsKey(name)) {
                commands.add(wrappedEventCommand(eventMap[name]!!))
            }
        }

        return when (stopEvent.executionBehavior) {
            PathPlannerTrajectory.StopEvent.ExecutionBehavior.SEQUENTIAL -> Commands.sequence(*commands.toTypedArray())

            PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL -> Commands.parallel(*commands.toTypedArray())

            PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE -> {
                val deadline: Command =
                    if (eventMap.containsKey(stopEvent.names[0])) wrappedEventCommand(
                        eventMap[stopEvent.names[0]]!!
                    ) else Commands.none()
                Commands.deadline(deadline, *commands.toTypedArray())
            }

            else -> throw java.lang.IllegalArgumentException(
                "Invalid stop event execution behavior: " + stopEvent.executionBehavior
            )
        }
    }

    private fun wrappedEventCommand(eventCommand: Command): CommandBase {
        return FunctionalCommand(
            { eventCommand.initialize() },
            { eventCommand.execute() },
            { interrupted: Boolean? -> eventCommand.end(interrupted!!) },
            { eventCommand.isFinished },
            *eventCommand.requirements.toTypedArray())
    }

}
