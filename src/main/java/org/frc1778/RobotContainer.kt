package org.frc1778


import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.commands.BalanceCommand
import org.frc1778.commands.DriveToChargeStation
import org.frc1778.commands.IntakeSpitCommand
import org.frc1778.commands.IntakeStopCommand
import org.frc1778.commands.IntakeSuckCommand
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.sequential
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
    //<editor-fold desc="Old Auto Code ">
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


    val autoPathConstraints = PathConstraints(
        2.75, // m/s
        1.875 //m/s^2
    )


    //<editor-fold desc="Old Auto Things">
    private val eventMap: HashMap<String, Command> = hashMapOf(
        "Spit Out Game Piece" to IntakeSpitCommand().withTimeout(.625),
        "Lower Intake" to IntakeSuckCommand(),
        "Pick Up Intake" to IntakeStopCommand(),
    )


    val station1Blue = {
        Drive.followTrajectoryGroupWithCommands(
            getAlliancePathGroup("Station 1 Red No Balance", autoPathConstraints, Alliance.Red, Alliance.Blue), eventMap
        )
    }

    val station2 = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 2", autoPathConstraints, Alliance.Red), eventMap
            )
            +BalanceCommand()

        }
    }


    val station3WithMarkers = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 3 Red No Balance", autoPathConstraints, Alliance.Red), eventMap
            )
            +IntakeStopCommand()
            // +BalanceCommand()
        }
    }

    val station3Score2 = {
        Drive.followTrajectoryGroupWithCommands(
            getAlliancePathGroup("Station 3 Score 2", autoPathConstraints, Alliance.Red), eventMap
        )
    }

    val station1Red = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                PathPlanner.loadPathGroup("Station 1 Red No Balance", autoPathConstraints), eventMap
            )
//            +BalanceCommand()
        }
    }

    val station1RedScore2 = {
        Drive.followTrajectoryGroupWithCommands(
            PathPlanner.loadPathGroup("Station 1 Red Score 2", autoPathConstraints), eventMap
        )

    }

    val station1RedScore2ThenBalance = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                PathPlanner.loadPathGroup("Station 1 Red Score 2 And Balance", autoPathConstraints), eventMap
            )
            +BalanceCommand()
        }
    }

    val station2Balance = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 2 Balance", autoPathConstraints, Alliance.Red), eventMap
            )
            +BalanceCommand()
        }
    }


    /**
     * A enumeration of the available autonomous modes.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    enum class AutoMode(val optionName: String, val command: Source<Command>) {
        STATION_ONE_BLUE("Station 1 Blue", station1Blue), STATION_ONE_RED(
            "Station 1 Red", station1Red
        ),
        STATION_ONE_RED_SCORE_2(
            "Station 1 Red Score 2", station1RedScore2
        ),
        STATION_ONE_RED_SCORE2_AND_BALANCE(
            "Station 1 Red Score 2 And Balance", station1RedScore2ThenBalance
        ),
        STATION_TWO("Red Station 2", station2), STATION_THREE_WITH_MARKERS(
            "Station 3",
            station3WithMarkers
        ),
        STATION_TWO_BALANCE("Station Two Balance", station2Balance),
        STATION_THREE_SCORE_2("Station 3 Score 2", station3Score2),



        ; //!Don't remove

        companion object {
            /** The default auto mode. */
            val default = STATION_ONE_RED
        }
    }


    //TODO: This might still be broken.
    //!!NEED TO GET THIS WORKING!!
    private val autoModeChooser = SendableChooser<AutoMode>().apply {
        AutoMode.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(AutoMode.default.optionName, AutoMode.default)
    }

    private val balanceChooser = SendableChooser<Boolean>().apply {
        addOption("Yes", true)
        addOption("No", false)
        setDefaultOption("No", false)
    }

    init {
        SmartDashboard.putData("Auto choices", autoModeChooser)
        SmartDashboard.putData("Balance?", balanceChooser)
    }


    fun getAutonomousCommand(): Command {
        return sequential {
            +autoModeChooser.selected!!.command()
            if (balanceChooser.selected!!) {
                +DriveToChargeStation()
                +BalanceCommand()
            }

        }
    }


    private fun getAlliancePathGroup(
        name: String, constraints: PathConstraints, from: Alliance, to: Alliance = Robot.alliance
    ) = PathPlanner.loadPathGroup(name, constraints).map {
        PathPlannerTrajectory.transformTrajectoryForAlliance(it, from, to)
    }
}