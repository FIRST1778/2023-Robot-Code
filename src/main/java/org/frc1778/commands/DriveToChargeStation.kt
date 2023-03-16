package org.frc1778.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Robot
import org.frc1778.RobotContainer
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPoint
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import kotlin.math.abs

class DriveToChargeStation : FalconCommand(Drive) {

    private lateinit var trajectoryTrackerCommand: SwerveTrajectoryTrackerCommand

    override fun initialize() {
        val holonomicRotation = Rotation2d.fromDegrees(if (abs(Drive.robotPosition.rotation.degrees) <= 90.0) 0.0 else 180.0)

        val balancePoint =
            if (Robot.alliance == DriverStation.Alliance.Red) BalanceLocation.RED_BALANCE else BalanceLocation.BLUE_BALANCE
        val entryPoint = BalanceLocation.values().first { it.qualifier.contains(Drive.robotPosition.translation) }
        val trajectory = PathPlanner.generatePath(
            RobotContainer.autoPathConstraints, listOf(
                PathPoint.fromCurrentHolonomicState(
                    Drive.robotPosition, Drive.kinematics.toChassisSpeeds(*Drive.swerveModuleStates().toTypedArray())
                ).withControlLengths(0.025, 0.025),
                PathPoint(
                    entryPoint.location, entryPoint.heading, holonomicRotation
                ).withControlLengths(.1, .5),
                PathPoint(balancePoint.first, balancePoint.second, holonomicRotation).withPrevControlLength(.1)
            )
        )
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

    enum class BalanceLocation(val location: Translation2d, val heading: Rotation2d, val qualifier: Rectangle2d) {
        BLUE_INNER(

            Translation2d(2.15, 2.8), Rotation2d.fromDegrees(0.0), Rectangle2d(
                Translation2d(1.55, 5.25), Translation2d(2.6, .15)
            )
        ),
        BLUE_OUTER(

            Translation2d(5.55, 2.8), Rotation2d.fromDegrees(180.0), Rectangle2d(
                Translation2d(5.3, 7.90), Translation2d(8.3, .15)
            )
        ),
        RED_INNER(

            Translation2d(14.35, 2.8), Rotation2d.fromDegrees(180.0), Rectangle2d(
                Translation2d(15.0, 5.25), Translation2d(13.75, .15)
            )
        ),
        RED_OUTER(

            Translation2d(11.0, 2.8), Rotation2d.fromDegrees(0.0), Rectangle2d(
                Translation2d(12.0, 7.90), Translation2d(8.3, .15)
            )
        );

        companion object {
            val BLUE_BALANCE = Translation2d(3.9, 2.8) to Rotation2d.fromDegrees(0.0)

            val RED_BALANCE = Translation2d(12.65, 2.8) to Rotation2d.fromDegrees(0.0)

        }
    }

}