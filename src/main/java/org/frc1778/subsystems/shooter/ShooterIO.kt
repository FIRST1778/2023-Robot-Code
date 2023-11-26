package org.frc1778.subsystems.shooter

import org.ghrobotics.lib.junction.AutoLog
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt

interface ShooterIO {

    fun updateInputs(inputs: ShooterIOInputs)

    fun stop()

    fun setVoltage(voltage: SIUnit<Volt>)
    fun setLoaded(loaded: Boolean)
}


@AutoLog
open class ShooterIOInputs {
    var shooterVoltage: Double = 0.0
    var shooterLoaded: Boolean = true
    var limitSwitchReading: Boolean = false
}