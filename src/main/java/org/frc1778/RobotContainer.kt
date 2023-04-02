package org.frc1778


import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.drive.BalanceCommand
import org.frc1778.commands.drive.DriveToChargeStation
import org.frc1778.commands.intake.IntakeSpitCommand
import org.frc1778.commands.intake.IntakeStopCommand
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.commands.shooter.ShooterLoadCommand
import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.commands.shooter.ShooterSuckCommand
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.wrappers.networktables.ShuffleboardTabBuilder

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

    private val autoPathConstraints = PathConstraints(
        3.75, // m/s
        2.875 //m/s^2
    )

    private val slowAutoPathConstraints = PathConstraints(
        2.75, 1.875
    )
    private val eventMap: HashMap<String, Command> =
        hashMapOf("Spit Out Game Piece" to IntakeSpitCommand().withTimeout(.625),
            "Spit Out Game Piece Long" to IntakeSpitCommand().withTimeout(1.5),
            "Lower Intake" to IntakeSuckCommand(),
            "Suck To Shooter" to parallelDeadline(ShooterSuckCommand()) {
                +ShooterLoadCommand()
            }.withTimeout(4.0),
            "Pick Up Intake" to IntakeStopCommand(),
            "Angle To First Position" to ShooterAngleCommand(Level.None),// Initialized in the getAutonomousCommand() function
            "Angle To Second Position" to ShooterAngleCommand(Level.None),
            "Angle To Third Position" to ShooterAngleCommand(Level.None),
            "Lower Shooter" to ShooterAngleCommand(Level.None),
            "Shoot" to ShooterShootCommand().withTimeout(0.3))
    val station1 = {
        Drive.followTrajectoryGroupWithCommands(
            getAlliancePathGroup("Station 1", autoPathConstraints, Alliance.Red), eventMap
        )
    }

    val station1Score2 = {
        Drive.followTrajectoryGroupWithCommands(
            getAlliancePathGroup("Station 1 Score 2", autoPathConstraints, Alliance.Red), eventMap
        )

    }


    val station2 = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 2", slowAutoPathConstraints, Alliance.Red), eventMap
            )
        }
    }

    val station2Balance = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 2 Balance", slowAutoPathConstraints, Alliance.Red), eventMap
            )
            +BalanceCommand()
        }
    }

    val station3 = {
        sequential {
            +Drive.followTrajectoryGroupWithCommands(
                getAlliancePathGroup("Station 3", slowAutoPathConstraints, Alliance.Red), eventMap
            )
            +IntakeStopCommand()
            // +BalanceCommand()
        }
    }

    val station3Score2 = {
        Drive.followTrajectoryGroupWithCommands(
            getAlliancePathGroup("Station 3 Score 2", slowAutoPathConstraints, Alliance.Red), eventMap
        )
    }

    /**
     * A enumeration of the available autonomous modes.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    enum class AutoMode(val optionName: String, val command: Source<Command>) {
        STATION_ONE("Station 1", station1), STATION_ONE_SCORE_2(
            "Station 1 Score 2",
            station1Score2
        ),
        STATION_TWO("Station 2", station2), STATION_TWO_BALANCE(
            "Station 2 Leave & Balance",
            station2Balance
        ),
        STATION_THREE_WITH_MARKERS("Station 3", station3), STATION_THREE_SCORE_2("Station 3 Score 2", station3Score2),


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
    private val firstShootingLevelChooser = SendableChooser<Level>().apply {
        Level.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(Level.Bottom.optionName, Level.Bottom)
    }
    private val secondShootingLevelChooser = SendableChooser<Level>().apply {
        Level.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(Level.Bottom.optionName, Level.Bottom)
    }
    private val thirdShootingLevelChooser = SendableChooser<Level>().apply {
        Level.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(Level.Bottom.optionName, Level.Bottom)
    }
    private val balanceChooser = SendableChooser<Boolean>().apply {
        addOption("Yes", true)
        addOption("No", false)
        setDefaultOption("No", false)
    }
    private val balanceLocationChooser = SendableChooser<Boolean>().apply {
        addOption("Outside", true)
        addOption("Inside", false)
        setDefaultOption("Outside", true)
    }

    init {
//        SmartDashboard.putData("Auto choices", autoModeChooser)
//        SmartDashboard.putData("Balance?", balanceChooser)
        val autoTab = ShuffleboardTabBuilder("Auto")
        autoTab.sendableChooser("Auto", autoModeChooser) {
            position(0, 0)
            size(2, 1)
        }
        autoTab.sendableChooser("First Shooting Level", firstShootingLevelChooser) {
            position(4, 0)
            size(2, 1)
        }
        autoTab.sendableChooser("Second Shooting Level", secondShootingLevelChooser) {
            position(4, 0)
            size(2, 1)
        }
        autoTab.sendableChooser("Third Shooting Level", thirdShootingLevelChooser) {
            position(4, 0)
            size(2, 1)
        }
        autoTab.sendableChooser("Balance?", balanceChooser) {
            position(2, 0)
            size(2, 1)
        }
        autoTab.sendableChooser("Balance Location", balanceLocationChooser) {
            position(4, 0)
            size(2, 1)
        }
    }


    fun getAutonomousCommand(): Command {
        eventMap["Angle To First Position"] = ShooterAngleCommand(firstShootingLevelChooser.selected)
        eventMap["Angle To Second Position"] = ShooterAngleCommand(secondShootingLevelChooser.selected)
        eventMap["Angle To Third Position"] = ShooterAngleCommand(thirdShootingLevelChooser.selected)
        return sequential {
            +autoModeChooser.selected!!.command()
            if (balanceChooser.selected!!) {
                +DriveToChargeStation(balanceLocationChooser.selected)
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