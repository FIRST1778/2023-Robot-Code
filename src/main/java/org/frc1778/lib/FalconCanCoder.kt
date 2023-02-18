package org.frc1778.lib

import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import edu.wpi.first.util.sendable.SendableBuilder
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms

class FalconCanCoder<K : SIKey>(
    canId: Int,
    model: NativeUnitModel<K>,
    offsetAngle: Double = 0.0
) : AbstractFalconAbsoluteEncoder<K>(model) {

    private val canCoder = CANCoder(canId).apply {
        configMagnetOffset(Math.toDegrees(offsetAngle))
        configSensorDirection( false)
        setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100)
    }
    override val rawPosition: SIUnit<NativeUnit> = canCoder.position.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity> = canCoder.velocity.nativeUnitsPer100ms
    override val absolutePosition: SIUnit<Radian> get() = canCoder.absolutePosition.degrees

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        canCoder.position = newPosition.value
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder?.addDoubleProperty(
            "Absolute Position",
            {
                absolutePosition.inDegrees()
            },
            {  }

        )
    }

}