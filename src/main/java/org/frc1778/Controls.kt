package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import org.frc1778.commands.shooter.ShooterLoadCommand
import org.frc1778.commands.drive.BalanceCommand
import org.frc1778.commands.intake.IntakeLineBreakOverrideCommand
import org.frc1778.commands.intake.IntakeLowerCommand
import org.frc1778.commands.intake.IntakeSpitCommand
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.commands.shooter.ShooterSuckCommand
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.wrappers.hid.mapControls
import kotlin.math.abs
import kotlin.math.withSign


object Controls {
    //TODO: Update to use a more personalized HID to go with new commands
    val driverControllerGenericHID = Joystick(0)
    private val XboxController = Joystick(1)

//    private val runIntakeCommand = RunIntake()

    val driverController = driverControllerGenericHID.mapControls {
        //TODO: Find Correct Button ID
        button(1) {
            whileOn {
//                Drive.gamePiecePlacementTrajectoryFollowCommand?.schedule()
            }
            changeOff {
//                Drive.gamePiecePlacementTrajectoryFollowCommand?.cancel()
            }
        }
    }

    val xboxController = XboxController.mapControls {
        button(1) { changeOn(ShooterAngleCommand(Level.Bottom)) }
        button(2) { change(BalanceCommand()) }
        button(3) { changeOn(ShooterAngleCommand(Level.Middle)) }
        button(4) { changeOn(ShooterAngleCommand(Level.Top)) }
        button(7) { changeOn(IntakeLineBreakOverrideCommand()) }
        button(8) { changeOn(ShooterAngleCommand(Level.THREE_POINT)) }

        axisButton(2, 0.7) {
            change(ShooterShootCommand())
            changeOff { ShooterAngleCommand(Level.None).schedule() }
        }
        axisButton(3, 0.7) { change(
            sequential {
                +ShooterAngleCommand(Level.None)
                +parallelDeadline(ShooterSuckCommand()) {
                    +ShooterLoadCommand()
                }
            }
        ) }
    }

    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
