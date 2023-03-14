package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.commands.*
import org.frc1778.lib.GameObject
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.DotStar
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters
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
            changeOn {
//                Robot.scoringStation = Station.Left
            }
        }// station 1

        button(2) {
            changeOn {
//                Robot.scoringStation = Station.Center
            }
        }// station 2
        button(3) {
            changeOn {
//                Robot.scoringStation = Station.Right
            }
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
            change(IntakeSpitCommand())
        } // intake spit
    }
    val operatorControllerBlue = operatorControllerGenericHID2.mapControls {
        // bug fix buttons
//        button(1) {changeOn(ZeroExtensionCommand()) } // reset extension
        button(2) {}
        button(3) {changeOn(sequential{+BalanceCommand()})}// other
        // level of placement
        button(4) {
//            changeOn {
////                Robot.scoringLevel = Level.Bottom
////                ArmAngleCommand(40.0.degrees).schedule()
//            }
        }// bottom
        button(5) {
//            changeOn (
//                Robot.scoringLevel = Level.Middle
//                sequential {
//                    +ConditionalCommand(ArmExtensionCommand(0.5.meters), WaitCommand(0.0), {Arm.getCurrentExtension() > 0.52.meters})
//                    +ArmAngleCommand(85.0.degrees)
//                    +ArmExtensionCommand(0.5.meters)
//                }
//            )
        }// middle
        button(6) {
//            changeOn (sequential {
////                Robot.scoringLevel = Level.Top
//                +ArmExtensionCommand(0.0.meters)
//                +ArmAngleCommand(100.0.degrees)
//                +ArmExtensionCommand(0.9.meters)
//            })
        }// top
        // manipulator open/close toggle
        button(7) {
//            changeOn(InstantCommand({
//                Manipulator.toggleState()
//            }))
        }

        // toggle manipulator
        button(8) {
//            changeOn(ArmToHopperCommand())
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
        button(10) // balance
        axisButton(0, -1.0) {
            changeOn {
                Robot.scoringSide = Side.Left
            }
            changeOff {
                Robot.scoringSide = Side.Right
            }
        }// left/right (cone mode)
    }


    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
