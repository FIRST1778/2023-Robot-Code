package org.frc1778.subsystems.shooter

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt

class ShooterIOSim : ShooterIO {
    override fun updateInputs(inputs: ShooterIOInputs) {
        inputs.shooterLoaded = true
    }

    override fun stop() {}

    override fun setVoltage(voltage: SIUnit<Volt>) {}

    override fun setLoaded(loaded: Boolean) {}

}