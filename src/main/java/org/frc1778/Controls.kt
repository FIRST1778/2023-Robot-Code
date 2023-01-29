package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.ghrobotics.lib.wrappers.hid.mapControls


object Controls {
    //TODO: Update to use a more personalized HID to go with new commands
    private val driverControllerGenericHID = Joystick(0)
    private val operatorControllerGenericHID = Joystick(2)


//    private val runIntakeCommand = RunIntake()


    val driverController = driverControllerGenericHID.mapControls {
//        button(1) {
//            change(Aim())
//        }
    }

    val operatorController = operatorControllerGenericHID.mapControls {
        button(1){
            changeOn(
                InstantCommand( {
                    Robot.sol.state = FalconSolenoid.State.Forward
                })
            )
        }
        button(2) {
            changeOn(
                InstantCommand( {
                    Robot.sol.state = FalconSolenoid.State.Reverse
                })
            )
        }
    }
}
