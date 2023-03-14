package org.frc1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.commands.IntakeSpitCommand
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Drive.positions
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


    /**
     * A enumeration of the available autonomous modes.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    enum class AutoMode(val optionName: String, val command: Source<Command>) {
        CUSTOM_AUTO_1("Blue Drive Station 1", driveStation1), CUSTOM_AUTO_2(
            "Blue Drive Station 2", driveStation2
        ),
        CUSTOM_AUTO_3("Blue Drive Station 3", driveStation3), CUSTOM_AUTO_4(
            "Red Drive Station 1", driveStation4
        ),
        CUSTOM_AUTO_5("Red Drive Station 2", driveStation5), CUSTOM_AUTO_6(
            "Red Drive Station 3", driveStation6
        );

        companion object {
            /** The default auto mode. */
            val default = CUSTOM_AUTO_1
        }
    }

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