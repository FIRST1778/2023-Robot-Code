package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.frc1778.lib.AbstractFalconAbsoluteEncoder
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms

class ShooterAbsoluteEncoder(sparkMax: CANSparkMax, model: NativeUnitRotationModel) : AbstractFalconAbsoluteEncoder<Radian>(model),
    Sendable {
    private val dutyCycleEncoder: SparkMaxAbsoluteEncoder = sparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    //Same as Position
    override val absolutePosition: SIUnit<Radian>
        get() = position
    override val rawPosition: SIUnit<NativeUnit>
        get() = dutyCycleEncoder.position.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = dutyCycleEncoder.velocity.nativeUnitsPer100ms

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        dutyCycleEncoder.zeroOffset = newPosition.value
    }

    fun setInverted(inverted: Boolean) {
        dutyCycleEncoder.inverted = inverted
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!.addDoubleProperty(
            "Angle Absolute Position",
            { absolutePosition.inDegrees() },
            {}
        )
        builder.addDoubleProperty(
            "Duty Cycle Output",
            { dutyCycleEncoder.position },
            {}
        )
    }

}