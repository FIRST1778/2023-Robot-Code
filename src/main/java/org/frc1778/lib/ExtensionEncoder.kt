package org.frc1778.lib

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.frc1778.Constants
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.motors.AbstractFalconEncoder
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.motors.rev.FalconMAXEncoder

class ExtensionEncoder(val extensionEncoder: FalconMAXEncoder<Meter>): Sendable{
    val position: SIUnit<Meter>
        get() = extensionEncoder.position
    val velocity: SIUnit<Velocity<Meter>>
        get() = extensionEncoder.velocity

    fun resetPosition(newPosition: SIUnit<Meter>) {
        extensionEncoder.resetPosition(newPosition)
    }
    override fun initSendable(builder: SendableBuilder?){
        builder!!.addDoubleProperty(
            "Extension Position",
            {-> position.value},
            {}
        )
    }
}