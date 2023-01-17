package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import org.ghrobotics.lib.wrappers.hid.mapControls


object Controls {
    //TODO: Update to use a more personalized HID to go with new commands
    private val driverControllerGenericHID = Joystick(0)
    private val operatorControllerGenericHID = Joystick(1)


//    private val runIntakeCommand = RunIntake()


    val driverController = driverControllerGenericHID.mapControls {
//        button(1) {
//            change(Aim())
//        }
    }
}
