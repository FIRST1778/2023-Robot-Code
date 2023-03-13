package org.frc1778.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.frc1778.Constants
import org.frc1778.lib.AbstractFalconAbsoluteEncoder
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class ManipulatorAbsoluteEncoder : AbstractFalconAbsoluteEncoder<Radian>(
    Constants.ManipulatorConstants.DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL
){
    val dutyCycleEncoder = DutyCycleEncoder(6)

    private var zeroPoint: Double = 0.0

    var inverted = true

    //Same as Position
    override val absolutePosition: SIUnit<Radian>
        get() = position
    override val rawPosition: SIUnit<NativeUnit>
        get() = if (!inverted) {
            (dutyCycleEncoder.get() - zeroPoint).nativeUnits
        } else {
            (zeroPoint - dutyCycleEncoder.get()).nativeUnits
        }
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = throw Error("Cannot Get Velocity from duty Cycle Encoder")

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        zeroPoint = newPosition.value
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!.addDoubleProperty(
            "Angle Absolute Position",
            { absolutePosition.inDegrees() },
            {}
        )
        builder.addDoubleProperty(
            "Duty Cycle Output",
            {dutyCycleEncoder.get()},
            {}
        )
    }

}