package org.frc1778.lib

import com.ctre.phoenix.sensors.CANCoder
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.AbstractFalconEncoder
import kotlin.math.roundToInt

class FalconCanCoder<K : SIKey>(
    canId: Int,
    model: NativeUnitModel<K>
) : AbstractFalconEncoder<K>(model) {

    private val canCoder = CANCoder(canId)
    override val rawPosition: SIUnit<NativeUnit> = canCoder.position.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity> = canCoder.velocity.nativeUnitsPer100ms
    val absolutePosition: SIUnit<Radian> get() = canCoder.absolutePosition.degrees

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        canCoder.position = newPosition.value
    }

}