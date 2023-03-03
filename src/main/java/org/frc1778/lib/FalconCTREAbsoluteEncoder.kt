package org.frc1778.lib

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.util.sendable.SendableBuilder
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms

class FalconCTREAbsoluteEncoder<K : SIKey>(
    private val canTalonSRX: TalonSRX,
    model: NativeUnitModel<K>) : AbstractFalconAbsoluteEncoder<K>(model) {

    init {
        canTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10)
    }

    constructor(id: Int, model: NativeUnitModel<K>) : this (TalonSRX(id), model)

    /**
     * Absolute position | Same as [position]
     */
    override val absolutePosition: SIUnit<K>
        get() = position
    override val rawPosition: SIUnit<NativeUnit>
        get() = canTalonSRX.selectedSensorPosition.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = canTalonSRX.selectedSensorVelocity.nativeUnitsPer100ms

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        TODO("Not yet implemented")
    }

    override fun initSendable(builder: SendableBuilder?) {
        TODO("Not yet implemented")
    }
}