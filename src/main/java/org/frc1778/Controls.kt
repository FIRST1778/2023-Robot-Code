package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.commands.*
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.wrappers.FalconSolenoid
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
                Drive.gamePiecePlacementTrajectoryFollowCommand?.schedule()
            }
            changeOff {
                Drive.gamePiecePlacementTrajectoryFollowCommand?.cancel()
            }
        }
    }

    val operatorControllerRed = operatorControllerGenericHID1.mapControls {
        //TODO: Confirm these are the correct
        //charging station selection
        button(0) {
            changeOn {
                Robot.scoringStation = Station.Left
            }
        }// station 1

        button(1) {
           changeOn {
                Robot.scoringStation = Station.Center
            }
        }// station 2
        button(2) {
            changeOn {
                Robot.scoringStation = Station.Right
            }
        }// station 3
        // Intake

        //TODO: Change these two to Standard Commands
        // The functionality you are looking for in this command could be housed in a standard FalconCommand
        // That has the same init behavior but with the changeOff in the cancel portion of the command
        button(3) {
            whileOn { IntakeSuckCommand() }
            changeOff {
                sequential {
                    IntakeStopCommand()
                    IntakeToggleCommand(FalconSolenoid.State.Reverse)
                }
            } // intake suck
        }
        button(4) {
            whileOn { IntakeSpitCommand() }
            changeOff { IntakeStopCommand() }
        } // intake spit
    }
    val operatorControllerBlue = operatorControllerGenericHID2.mapControls {
        // bug fix buttons
        button(0) { change(ZeroExtensionCommand()) } // reset extension
        button(1) {
            changeOn(Robot.placeGameObjectCommand)
        }
        button(2) // other
        // level of placement
        button(3) {
            changeOn {
                Robot.scoringLevel = Level.Bottom
            }
        }// bottom
        button(4) {
            changeOn {
                Robot.scoringLevel = Level.Middle
            }
        }// middle
        button(5) {
            changeOn {
                Robot.scoringLevel = Level.Top
            }
        }// top
        // manipulator open/close toggle
        button(6) {
            change(
                if (Manipulator.manipulatorOpen) {
                    ManipulatorCloseCommand()
                } else {
                    ManipulatorOpenCommand()
                }
            )
        } // toggle manipulator
        button(7) {
            change(sequential {
                +IntakeToggleCommand(FalconSolenoid.State.Forward) // intake out
                +ExtensionCommand(0.0.meters)// zero
                +ArmAngleCommand(50.0.degrees) // hopper angle
                +ExtensionCommand(0.0.meters) // hopper extension
            })
        } // manipulator to hopper

        // switches
        button(10) {
            changeOn {
                Robot.gamePiece = GamePiece.Cone
            } // cone
            changeOff {
                Robot.gamePiece = GamePiece.Cube
            } // cube
        } // cone/cube
        button(9) // balance
        //TODO: Confirm these
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
