package org.frc1778.subsystems.shooter

import com.gamingnight.junction.AutoLog
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt

interface ShooterIO {

    fun updateInputs(inputs: ShooterIOInputs) {

    }

    fun stop() {

    }

    fun setVoltage(voltage: SIUnit<Volt>) {

    }

    fun setLoaded(loaded: Boolean) {

    }
}


@AutoLog
open class ShooterIOInputs {
    var shooterVoltage: Double = 0.0
    var shooterLoaded: Boolean = true
    var limitSwitchReading: Boolean = false
}

@AutoLog
open class TestIOInputs {
    var boolean = false
    var long = 0L
    var float = 0f
    var double = 0.0
    var string = ""

    var byteArray = listOf<Byte>()
    var booleanArray = listOf<Boolean>()
    var longArray = listOf<Long>()
    var floatArray = listOf<Float>()
    var doubleArray = listOf<Double>()
    var stringArray = listOf<String>()
}