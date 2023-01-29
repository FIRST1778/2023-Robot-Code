package org.frc1778.lib

import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.motors.AbstractFalconEncoder

abstract class AbstractFalconAbsoluteEncoder<K: SIKey>(model: NativeUnitModel<K>) : AbstractFalconEncoder<K>(model) {

    abstract val absolutePosition: SIUnit<Radian>

}