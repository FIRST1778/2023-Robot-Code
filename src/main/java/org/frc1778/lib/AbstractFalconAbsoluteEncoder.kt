package org.frc1778.lib

import edu.wpi.first.util.sendable.Sendable
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.motors.AbstractFalconEncoder

abstract class AbstractFalconAbsoluteEncoder<K : SIKey>(model: NativeUnitModel<K>) : AbstractFalconEncoder<K>(model),
    Sendable {

    abstract val absolutePosition: SIUnit<Radian>

}