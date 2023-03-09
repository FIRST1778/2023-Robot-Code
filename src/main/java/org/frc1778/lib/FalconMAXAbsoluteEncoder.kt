package org.frc1778.lib

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms

class FalconMAXAbsoluteEncoder(
    canSparkMax: CANSparkMax,
    type: SparkMaxAbsoluteEncoder.Type,
    model: NativeUnitModel<Radian>,
) : AbstractFalconAbsoluteEncoder<Radian>(model) {

    constructor(
        id: Int,
        motorType: CANSparkMaxLowLevel.MotorType,
        type: SparkMaxAbsoluteEncoder.Type,
        model: NativeUnitModel<Radian>
    ) : this(
        CANSparkMax(id, motorType), type, model
    )

    private val canAbsoluteEncoder = canSparkMax.getAbsoluteEncoder(type)
    public var simulatedPosition = 0.0.radians

    /**
     * Same as [position]
     */
    override val absolutePosition: SIUnit<Radian>
        get() {
            if(RobotBase.isSimulation()) {
                return simulatedPosition
            }
            return position
        }
    override val rawPosition: SIUnit<NativeUnit>
        get() = canAbsoluteEncoder.position.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = canAbsoluteEncoder.velocity.nativeUnitsPer100ms

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        throw Error("Unable to set position of Absolute Encoder")
    }

    fun setZeroOffset(offset: Double) {
        canAbsoluteEncoder.zeroOffset = offset
    }

    override fun initSendable(builder: SendableBuilder?) {
        TODO("Not yet implemented")
    }
}