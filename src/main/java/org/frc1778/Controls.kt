package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
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
//        button(1) {
//            changeOn(InstantCommand({
//                Drive.setNeutral()
//            }, Drive))
//        }
    }

    val operatorControllerRed = operatorControllerGenericHID1.mapControls {
        // charging station selection
        button(0){} // station 1
        button(1){} // station 2
        button(2){} // station 3
        // Intake
        button(3){} // intake suck
        button(4){} // intake spit
    }
    val operatorControllerBlue = operatorControllerGenericHID2.mapControls {
        // bug fix buttons
        button(0){} // reset extension
        button(1){} // other
        button(2){} // other
        // level of placement
        button(3){} // bottom
        button(4){} // middle
        button(5){} // top
        // manipulator open/close toggle
        button(6){} // toggle manipulator

        // switches
        button(10){} // cone/cube
        button(9){} // balance
        axisButton(0, -1.0 ){} // left/right (cone mode)
    }


    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if(abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
