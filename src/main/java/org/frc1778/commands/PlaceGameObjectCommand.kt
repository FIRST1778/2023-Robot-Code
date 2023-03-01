package org.frc1778.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.frc1778.Constants
import org.frc1778.GamePiece
import org.frc1778.Level
import org.frc1778.Robot
import org.frc1778.Side
import org.frc1778.Station
import org.frc1778.commands.SwerveTrajectoryTrackerCommand
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.hypot

class PlaceGameObjectCommand(
    var level: Level, private val gamePiece: GamePiece, private val station: Station, side: Side
) : FalconCommand(Arm) {

    lateinit var command: Command

    private val robotPose: Pose3d = station.ours()

    init {
        if (gamePiece == GamePiece.Cone) robotPose.transformBy(side.transform)
        Drive.scoringPose = robotPose.toPose2d()
        Drive.gamePiecePlacementTrajectoryFollowCommand = PlaceGameObjectDriveCommand()
    }

    override fun initialize() {
        val goalPose: Pose3d = station.ours()
            .transformBy(
                level.transform + Transform3d(
                    Translation3d(0.0, 0.0, gamePiece.heightOffset), Rotation3d()
                )
            )
        val armTranslation3d = Transform3d(
            robotPose.transformBy(Constants.ArmConstants.AORTransform), goalPose
        ).translation
        val armTranslation2d = Translation2d(armTranslation3d.y, armTranslation3d.z)
        val armRotation = 90.degrees + armTranslation2d.angle.radians.radians
        val armExtension = hypot(armTranslation2d.x, armTranslation2d.y).meters
        command = sequential {
            +ArmAngleCommand(armRotation)
            +ExtensionCommand(armExtension)
        }
        command.schedule()
    }

    override fun cancel() {
        super.cancel()
        command.cancel()
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }


}