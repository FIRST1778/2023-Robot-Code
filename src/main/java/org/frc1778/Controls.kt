package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import org.ghrobotics.lib.wrappers.hid.mapControls
import kotlin.math.abs
import kotlin.math.withSign


object Controls {
    //TODO: Update to use a more personalized HID to go with new commands
    private val driverControllerGenericHID = Joystick(0)
    private val operatorControllerGenericHID = Joystick(1)


//    private val runIntakeCommand = RunIntake()


    val driverController = driverControllerGenericHID.mapControls {
//        button(1) {
//            changeOn(InstantCommand({
//                Drive.setNeutral()
//            }, Drive))
//        }
    }

    val operatorController = operatorControllerGenericHID.mapControls {

    }

    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if(abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}
