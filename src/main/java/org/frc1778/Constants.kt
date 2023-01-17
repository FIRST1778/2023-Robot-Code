package org.frc1778

import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.units.Frac
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.meters

object Constants {

    object DriveConstants {
        val maxSpeed: SIUnit<Frac<Meter, Second>> = SIUnit(8.0)
        const val wheelBase: Double = 23.5
        const val trackWidth: Double = 23.5
        const val pigeonCanID: Int = 0
        val swerveConstants = SwerveModuleConstants()
    }
}