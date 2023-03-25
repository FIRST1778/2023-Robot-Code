package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import org.frc1778.commands.drive.BalanceCommand
import org.frc1778.commands.intake.IntakeLowerCommand
import org.frc1778.commands.intake.IntakeSpitCommand
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.commands.shooter.ShooterSuckCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
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

    val operatorControllerRed = operatorControllerGenericHID1.mapControls {
        //TODO: Confirm these are the correct
        //charging station selection
        button(1) {
            change(
                IntakeSpitCommand()
            )
        }// station 1

        button(2) {
            change(
                ShooterShootCommand()
            )
        }// station 2
        button(3) {
            changeOn(ShooterAngleCommand(Level.None))
            change(
                sequential {
                    +ShooterSuckCommand()
                }
            )
        }// station 3
        // Intake

        //TODO: Change these two to Standard Commands
        // The functionality you are looking for in this command could be housed in a standard FalconCommand
        // That has the same init behavior but with the changeOff in the cancel portion of the command
        button(4) {
            change(IntakeSuckCommand())
            // intake suck
        }
        button(5) {
            change(
                IntakeLowerCommand()
            )
        } // intake spit
    }
    val operatorControllerBlue = operatorControllerGenericHID2.mapControls {
        // bug fix buttons
//        button(1) {changeOn(ZeroExtensionCommand()) } // reset extension
        button(2) {}
//        button(3) {changeOn(sequential{+BalanceCommand()})}// other
        // level of placement
        button(4) {
            changeOn(ShooterAngleCommand(Level.Bottom))
        }// bottom
        button(5) {
            changeOn(ShooterAngleCommand(Level.Middle))
        }// middle
        button(6) {
            changeOn(ShooterAngleCommand(Level.Top))

        }// top
        // manipulator open/close toggle
        button(7) {
//            changeOn(InstantCommand({
//                Manipulator.toggleState()
//            }))
        }

        // toggle manipulator
        button(8) {
            changeOn(ShooterAngleCommand(Level.None))
        } // manipulator to hopper

        // switches
        button(11) {
            whileOn {
//                DotStar.display(DotStar.Color.Yellow)
//                Robot.gamePiece = GamePiece.Cone
//                Manipulator.setGameObject(GameObject.CONE)
            } // cone
            whileOff {
//                DotStar.display(DotStar.Color.Purple)
//                Robot.gamePiece = GamePiece.Cube
//                Manipulator.setGameObject(GameObject.CUBE)
            }

            // cube
        } // cone/cube
        button(10) {
            change(BalanceCommand())
        } // balance
        axisButton(0, -1.0) {

        }// left/right (cone mode)
    }


    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
