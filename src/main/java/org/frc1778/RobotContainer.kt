package org.frc1778

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.IntakeSpitCommand
import org.frc1778.commands.IntakeStopCommand
import org.frc1778.commands.IntakeSuckCommand
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.utils.Source

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {


    //<editor-fold desc="Old Auto Things">
    /**
    val driveStation1 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(1.85, 4.40), Rotation2d.fromDegrees(180.0)
    ), Drive.modules.positions.toTypedArray()
    )
    })
    +IntakeSpitCommand().withTimeout(7.5)
    }
    }

    val driveStation2 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(1.85, 2.7), Rotation2d.fromDegrees(180.0)
    ), Drive.modules.positions.toTypedArray()
    )
    +IntakeSpitCommand().withTimeout(7.5)

    }

    )
    }
    }

    val driveStation3 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(1.85, 1.05), Rotation2d.fromDegrees(180.0)
    ), Drive.modules.positions.toTypedArray()
    )
    +IntakeSpitCommand().withTimeout(7.5)

    })
    }
    }

    val driveStation4 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(14.75, 4.40), Rotation2d.fromDegrees(0.0)
    ), Drive.modules.positions.toTypedArray()
    )
    })
    +IntakeSpitCommand().withTimeout(7.5)
    }
    }

    val driveStation5 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(14.75, 2.7), Rotation2d.fromDegrees(0.0)
    ), Drive.modules.positions.toTypedArray()
    )

    })
    +IntakeSpitCommand().withTimeout(7.5)

    }

    }

    val driveStation6 = {
    sequential {
    +InstantCommand({
    Drive.resetPosition(
    Pose2d(
    Translation2d(14.75, 1.05), Rotation2d.fromDegrees(0.0)
    ), Drive.modules.positions.toTypedArray()
    )

    })
    +IntakeSpitCommand().withTimeout(7.5)

    }
    }
     */
    //</editor-fold>

    private val autoPathConstraints = PathConstraints(
        4.0, // m/s
        2.5 //m/s^2
    )

    val stationOne = {
        Drive.followTrajectory(PathPlanner.loadPath("station 1", autoPathConstraints))
    }


    //TODO: Test
    val stationOneWithMarkers = {
        Drive.followTrajectoryWithCommands(
            { PathPlanner.loadPath("station 1", autoPathConstraints) },
            hashMapOf(
                "Spit Out Game Piece" to IntakeSpitCommand().withTimeout(2.5),
                "Lower Intake" to IntakeSuckCommand(),
                "Pick Up Intake" to IntakeStopCommand(),
                //"Balance" to BalanceDriveCommand() TODO
                )
        )
    }

    /**
     * A enumeration of the available autonomous modes.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    enum class AutoMode(val optionName: String, val command: Source<Command>) {
        STATION_ONE("Station 1", stationOne),
        STATION_ONE_WITH_MARKERS("Station 1 With Markers", stationOneWithMarkers)


        ; //!Don't remove

        companion object {
            /** The default auto mode. */
            val default = STATION_ONE
        }
    }


    //TODO: This might still be broken.
    //!!NEED TO GET THIS WORKING!!
    private val autoModeChooser = SendableChooser<AutoMode>().apply {
        AutoMode.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(AutoMode.default.optionName, AutoMode.default)
    }

    init {
        configureBindings()
        SmartDashboard.putData("Auto choices", autoModeChooser)
    }


    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings() {

    }


    fun getAutonomousCommand(): Command {
        return autoModeChooser.selected?.command?.let { it() } ?: AutoMode.default.command()
    }
}