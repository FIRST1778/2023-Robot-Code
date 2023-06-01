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
    private val driverControllerGenericHID = Joystick(0)
    private val operatorControllerGenericHID1 = Joystick(1)
    private val operatorControllerGenericHID2 = Joystick(2)

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

    /*
    Operator Controls
    Red
        -1: Intake Spit
        -2: Shoot
        -3: Shooter Load Cube
        -4: Intake Suck
        -5: Lower Intake Wheels (for tipping cones)
    Blue
        -1: Line Break Override (if line break breaks)
        -2: Unbound Debug Button
        -3: Unbound Debug Button
        -4: Shooter Angle Bottom
        -5: Shooter Angle Middle
        -6: Shooter Angle Top
        -7: Shooter to Hopper
        -8: Shooter 3-Point (for shuttling)
        -9: Unbound Switch
        -10: Balance Switch
        -Axis 1: Unbound Switch
    */

    val operatorControllerRed = operatorControllerGenericHID1.mapControls {
        button(1) {
            change(
                IntakeSpitCommand()
            )
        }

        button(2) {
            change(
                ShooterShootCommand()
            )
            changeOff {
                ShooterAngleCommand(Level.None).schedule()
            }
        }
        button(3) {
            change(
                sequential {
                    +ShooterAngleCommand(Level.None)
                    +parallelDeadline(ShooterSuckCommand()) {
                        +ShooterLoadCommand()
                    }
                }
            )
        }

        button(4) {
            change(IntakeSuckCommand())
        }
        button(5) {
            change(
                IntakeLowerCommand()
            )
        }
    }
    val operatorControllerBlue = operatorControllerGenericHID2.mapControls {
        // bug fix buttons
        button(1) { changeOn(IntakeLineBreakOverrideCommand()) }
        button(2) {}
        button(3) {}
        button(4) {
            changeOn(
                ShooterAngleCommand(Level.Bottom)
            )
        }
        button(5) {
            changeOn(ShooterAngleCommand(Level.Middle))
        }
        button(6) {
            changeOn(ShooterAngleCommand(Level.Top))

        }
        button(7) {
            changeOn(ShooterAngleCommand(Level.None))
        }
        button(8) {
            changeOn(ShooterAngleCommand(Level.THREE_POINT))
        }
        button(9) {

        }
        button(11) {}
        button(10) { change(BalanceCommand()) }
        axisButton(1, -1.0) {}
    }


    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
