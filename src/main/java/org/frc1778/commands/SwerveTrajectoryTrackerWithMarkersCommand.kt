package org.frc1778.commands


import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.lib.FalconSwerveDrivetrain
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.FalconDashboard
import java.util.*

class SwerveTrajectoryTrackerWithMarkersCommand(
    private val drivetrain: FalconSwerveDrivetrain<*>,
    private val trajectory: PathPlannerTrajectory,
    private val eventMap: HashMap<String, Command>
) : FalconCommand(drivetrain) {


    private val currentCommands = mutableMapOf<Command, Boolean>()
    private val markers = trajectory.markers
    private val unpassedMarkers = mutableListOf<PathPlannerTrajectory.EventMarker>()


    private var isFinished = true
    private val timer = Timer()
    private var elapsed = 0.0
    private lateinit var trajectoryFollowingCommand: SwerveTrajectoryTrackerCommand

    init {
        for (marker in markers) {
            for (name in marker.names) {
                if (eventMap.containsKey(name)) {
                    val reqs = eventMap[name]!!.requirements
                    if (reqs.contains(drivetrain)) {
                        throw IllegalArgumentException(
                            "Events that are triggered during path following cannot require the drive subsystem"
                        )
                    }
                    m_requirements.addAll(reqs)
                }
            }
        }
    }

    override fun initialize() {
        drivetrain.setTrajectory(trajectory)

        currentCommands.clear()
        unpassedMarkers.clear()
        unpassedMarkers.addAll(markers)

        isFinished = false


        timer.reset()
        timer.start()

        trajectoryFollowingCommand = drivetrain.followTrajectory(trajectory)
        trajectoryFollowingCommand.initialize()
        currentCommands[trajectoryFollowingCommand] = true

        FalconDashboard.isFollowingPath = true
    }

    override fun execute() {
        for ((command, runCommand) in currentCommands) {
            if (!runCommand) {
                continue
            }

            command.execute()

            if (command.isFinished) {
                command.end(false)
                currentCommands[command] = false
                if (command == trajectoryFollowingCommand) {
                    isFinished = true
                }
            }
        }

        elapsed = timer.get()

        if (unpassedMarkers.size > 0 && elapsed >= unpassedMarkers[0].timeSeconds) {
            val marker = unpassedMarkers.removeAt(0)
            for (name in marker.names) {
                if (eventMap.containsKey(name)) {
                    val eventCommand = eventMap[name]!!

                    for ((command, runCommand) in currentCommands) {
                        if (!runCommand) {
                            continue
                        }
                        if (!Collections.disjoint(
                                command.requirements, eventCommand.requirements
                            )
                        ) {
                            command.end(true)
                            currentCommands[command] = false
                        }
                    }

                    eventCommand.initialize()
                    currentCommands[eventCommand] = true
                }
            }
        }


    }

    override fun end(interrupted: Boolean) {
        for ((command, run) in currentCommands) {
            if (run) {
                command.end(true)
            }
        }
    }

    override fun isFinished(): Boolean {
        return isFinished
    }





}