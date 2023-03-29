package org.frc1778.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.frc1778.Robot
import org.frc1778.lib.SwerveTrajectoryTrackerCommand
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPoint
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.abs

class DriveToChargeStation(private val outerBalance: Boolean) : FalconCommand(Drive) {
    companion object {
        private val pathConstraints = PathConstraints(3.5, 3.0)
    }

    private lateinit var trajectoryTrackerCommand: SwerveTrajectoryTrackerCommand

    override fun initialize() {
        val holonomicRotation =
            Rotation2d.fromDegrees(if (abs(Drive.robotPosition.rotation.degrees) <= 90.0) 0.0 else 180.0)

        val balancePoint =
            if (Robot.alliance == Alliance.Red) BalanceLocation.RED_BALANCE else BalanceLocation.BLUE_BALANCE
        val entryPoint = BalanceLocation.getLocationFromAlliance(Robot.alliance)
            .let { if (outerBalance) it.outerPoint else it.innerPoint }
        val trajectory = if (Drive.robotPosition.translation in entryPoint.qualifier) {
            PathPlanner.generatePath(
                pathConstraints, listOf(
                    PathPoint.fromCurrentHolonomicState(
                        Drive.robotPosition,
                        Drive.kinematics.toChassisSpeeds(*Drive.swerveModuleStates().toTypedArray())
                    ).withControlLengths(0.025, 0.025),
                    PathPoint(
                        entryPoint.location, entryPoint.heading, holonomicRotation, 3.0
                    ).withControlLengths(.2, .5),
                    PathPoint(balancePoint.first, balancePoint.second, holonomicRotation).withPrevControlLength(.1)
                )
            )
        } else {

            val points = StationBalancingPaths.values().first {
                val qualifier = it.qualifier.transformForAlliance(
                    Robot.alliance, Alliance.Blue
                )
                Drive.robotPosition.translation in qualifier
            }.points.map {(location, heading) ->
                PathPoint(
                    location, heading, holonomicRotation
                ).withControlLengths(.125, .25).transformForAlliance(
                   Robot.alliance, Alliance.Blue
                )
            }

            PathPlanner.generatePath(
                pathConstraints, mutableListOf(
                    PathPoint.fromCurrentHolonomicState(
                        Drive.robotPosition, Drive.kinematics.toChassisSpeeds(
                            *Drive.swerveModuleStates().toTypedArray()
                        )
                    ),
                    *points.toTypedArray(),
                    PathPoint(
                        entryPoint.location, entryPoint.heading, holonomicRotation, 3.0
                    ).withControlLengths(.2, .5),
                    PathPoint(balancePoint.first, balancePoint.second, holonomicRotation).withPrevControlLength(.1)
                )
            )
        }
        trajectoryTrackerCommand = Drive.followTrajectory { trajectory }
        trajectoryTrackerCommand.initialize()
    }

    override fun execute() {
        trajectoryTrackerCommand.execute()
    }

    override fun cancel() {
        super.cancel()
        trajectoryTrackerCommand.cancel()
    }

    override fun end(interrupted: Boolean) {
        trajectoryTrackerCommand.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return trajectoryTrackerCommand.isFinished
    }

    //

    enum class BalanceLocation(val innerPoint: BalancePoint, val outerPoint: BalancePoint) {
        BLUE(
            innerPoint = BalancePoint(
                Translation2d(1.9, 2.8), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0), Rectangle2d(
                    Translation2d(1.55, 5.25), Translation2d(2.6, .15)
                )
            ),


            outerPoint = BalancePoint(
                Translation2d(6.15, 2.8), Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(1800.0), Rectangle2d(
                    Translation2d(5.3, 7.90), Translation2d(8.3, .15)
                )
            )
        ),
        RED(
            innerPoint = BalancePoint(
                Translation2d(14.35, 2.8), Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(180.0), Rectangle2d(
                    Translation2d(15.0, 5.25), Translation2d(13.75, .15)
                )
            ),

            outerPoint = BalancePoint(
                Translation2d(10.3, 2.8), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0), Rectangle2d(
                    Translation2d(12.0, 7.90), Translation2d(8.3, .15)
                )
            )
        );


        companion object {
            fun getLocationFromAlliance(alliance: Alliance): BalanceLocation {
                return if (alliance == Alliance.Red) RED else BLUE
            }

            val BLUE_BALANCE = Translation2d(4.0, 2.8) to Rotation2d.fromDegrees(0.0)

            val RED_BALANCE = Translation2d(12.55, 2.8) to Rotation2d.fromDegrees(0.0)

        }
    }

    enum class StationBalancingPaths(val points: List<Pair<Translation2d, Rotation2d>>, val qualifier: Rectangle2d) {
        STATION_ONE(
            listOf(
                Translation2d(2.1, .7) to Rotation2d.fromDegrees(0.0),
                Translation2d(3.9, .7) to Rotation2d.fromDegrees(0.0),
                Translation2d(5.5, .95) to Rotation2d.fromDegrees(70.0)
            ), Rectangle2d(
                Translation2d(1.45, 2.7), Translation2d(2.85, 0.0)
            )
        ),
        STATION_THREE(
            listOf(
                Translation2d(2.1, 4.75) to Rotation2d.fromDegrees(0.0),
                Translation2d(3.9, 4.75) to Rotation2d.fromDegrees(0.0),
                Translation2d(5.5, 4.5) to Rotation2d.fromDegrees(-70.0)
            ), Rectangle2d(
                Translation2d(1.45, 2.7), Translation2d(3.25, 5.4)
            )
        )
    }

    data class BalancePoint(
        val location: Translation2d,
        val heading: Rotation2d,
        val holonomicRotation: Rotation2d,
        val qualifier: Rectangle2d
    )

    private fun Rectangle2d.transformForAlliance(to: Alliance, from: Alliance): Rectangle2d {
        if (to != from) {
            val transformDirection: Int = if (to == Alliance.Red) {
                1
            } else {
                -1
            }
            return Rectangle2d(
                ((16.54 / 2) + (transformDirection * (abs((16.54 / 2) - x.value))) - (transformDirection * w.value)).meters , y, w, h
            )
        }
        return this
    }

    private fun PathPoint.transformForAlliance(to: Alliance, from: Alliance): PathPoint {
        if (to != from) {
            val transformDirection: Int = if (to == Alliance.Red) {
                1
            } else {
                -1
            }
            val currentPosition = this.position
            val transformedPosition = Translation2d((16.54 / 2) + (transformDirection * abs((16.54 / 2) - currentPosition.x)), currentPosition.y)


            val currentRotation = this.heading
            val transformedRotation =
                currentRotation.rotateBy(Rotation2d.fromDegrees(180.0)) * -1.0

            return PathPoint(
                transformedPosition, transformedRotation, this.holonomicRotation, this.velocityOverride
            ).withControlLengths(this.prevControlLength, this.nextControlLength)

        }
        return this
    }

}