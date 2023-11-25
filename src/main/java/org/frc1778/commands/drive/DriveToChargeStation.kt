package org.frc1778.commands.drive

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.frc1778.Level
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.subsystems.drive.Drive
import org.frc1778.subsystems.wrist.Wrist
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.derived.degrees

class DriveToChargeStation(private val outerBalance: Boolean) : FalconCommand(Drive) {
    private lateinit var command: Command

    override fun initialize() {
        command = parallel {
            if(Wrist.getCurrentAngle() > 95.degrees) {
                +ShooterAngleCommand(Level.None)
            }
            StationBalancingPaths.entries.firstOrNull {
                Drive.robotPosition.translation in it.qualifier
            }?.let {
                +sequential {
                    +AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile(if (outerBalance) it.pathNames.first else it.pathNames.second),
                        Drive.pathConstraints,
//                    Double.MAX_VALUE // Rotation Delay Distance. We don't want to rotate so big number
                    )
                    +AutoBuilder.followPathWithEvents(
                        PathPlannerPath.fromPathFile(if (outerBalance) it.pathNames.first else it.pathNames.second),
                    )
                }
            } ?: +Commands.none()

        }
        command.initialize()

    }

    override fun execute() {
        command.execute()
    }

    override fun cancel() {
        super.cancel()
        command.cancel()
    }

    override fun end(interrupted: Boolean) {
        command.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }
    enum class StationBalancingPaths(
        val pathNames: Pair<String, String>, val qualifier: Rectangle2d
    ) {
        STATION_THREE(
            "" to "", Rectangle2d(
                Translation2d(1.45, 2.7), Translation2d(2.85, 0.0)
            )
        ),
        STATION_ONE(
            "Outer Balance Station 1" to "", Rectangle2d(
                Translation2d(1.45, 2.7), Translation2d(3.25, 5.4)
            )
        );

    }

}