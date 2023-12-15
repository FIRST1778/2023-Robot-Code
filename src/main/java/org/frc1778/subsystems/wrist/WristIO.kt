package org.frc1778.subsystems.wrist

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import org.frc1778.Constants
import org.frc1778.subsystems.ShooterAbsoluteEncoder
import org.ghrobotics.lib.junction.AutoLog
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.falconMAX
import org.littletonrobotics.junction.LoggedRobot
import kotlin.math.PI

interface WristIO {

    fun updateInputs(inputs: WristInputs)
    fun getCurrentAngle(): SIUnit<Radian>
    fun setVoltage(voltage: SIUnit<Volt>)

    fun resetPosition(angle: SIUnit<Radian>)

    var brakeMode: Boolean

    val brakeModeSwitch: Boolean

}

@AutoLog
open class WristInputs {
    var angleMotorVoltage: SIUnit<Volt> = 0.volts
    var angleMotorCurrent: SIUnit<Ampere> = 0.amps
    var currentAngle: SIUnit<Radian> = 0.radians
    var velocity: SIUnit<AngularVelocity> = 0.radians / 1.seconds
}

class WristIOSparkMax() : WristIO {
    val _brakeModeSwitch = DigitalInput(3)

    val angleMotor = falconMAX(
        Constants.ShooterConstants.ANGLE_MOTOR_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        if (LoggedRobot.isReal()) Constants.ShooterConstants.ANGLE_MOTOR_UNIT_MODEL else NativeUnitRotationModel((2 * PI).nativeUnits),
    ) {
        brakeMode = true
        outputInverted = true
        canSparkMax.encoder.position = 0.0
        encoder.resetPosition(90.degrees)
    }

    init {
        angleMotor.canSparkMax.let{

        }
    }

    val encoder = ShooterAbsoluteEncoder(angleMotor.canSparkMax, NativeUnitRotationModel(1.nativeUnits)).apply {
        setInverted(true)
        resetPosition((210.75 - 90.0).degrees)
    }

    override fun updateInputs(inputs: WristInputs) {
        inputs.run {
            angleMotorVoltage = angleMotor.voltageOutput
            angleMotorCurrent = angleMotor.drawnCurrent
            currentAngle = getCurrentAngle()
        }
    }

    override fun getCurrentAngle(): SIUnit<Radian> {
        return encoder.absolutePosition
    }

    override fun setVoltage(voltage: SIUnit<Volt>) {
        angleMotor.setVoltage(voltage)
    }

    override fun resetPosition(angle: SIUnit<Radian>) {
        angleMotor.encoder.resetPosition(angle)
    }

    override var brakeMode: Boolean
        get() = angleMotor.brakeMode
        set(v) {
            angleMotor.brakeMode = brakeMode
        }
    override val brakeModeSwitch: Boolean
        get() = _brakeModeSwitch.get()


}

//class WristIOSim() : WristIO {
//    val _brakeModeSwitch = DigitalInput(3)
//
//    val angleMotor = falconMAX(
//        Constants.ShooterConstants.ANGLE_MOTOR_ID,
//        CANSparkMaxLowLevel.MotorType.kBrushless,
//        if (LoggedRobot.isReal()) Constants.ShooterConstants.ANGLE_MOTOR_UNIT_MODEL else NativeUnitRotationModel((2 * PI).nativeUnits),
//    ) {
//        brakeMode = true
//        outputInverted = true
//        canSparkMax.encoder.position = 0.0
//        encoder.resetPosition(90.degrees)
//    }
//
//    val revMotorSim = REVPhysicsSim().apply {
//        addSparkMax(
//            angleMotor.canSparkMax, DCMotor.getNEO(1).withReduction(1.0 / ((24.0 / 64.0) * (1.0 / 5.0) * (1.0 / 4.0)))
//        )
//    }
//
//    override fun updateInputs(inputs: WristInputs) {
//        //This function is called periodically so we can call simulation periodic stuff in here
//        simulationPeriodic()
//        inputs.run {
//            angleMotorVoltage = angleMotor.voltageOutput
//            angleMotorCurrent = angleMotor.drawnCurrent
//            currentAngle = getCurrentAngle()
//        }
//    }
//
//    override fun getCurrentAngle(): SIUnit<Radian> {
//        return MathUtil.inputModulus(
//            angleMotor.encoder.position.value, 0.0, 2.0 * PI
//        ).coerceIn(90.degrees.value, 300.degrees.value).radians
//    }
//
//    override fun setVoltage(voltage: SIUnit<Volt>) {
//        angleMotor.setVoltage(voltage)
//    }
//
//    override fun resetPosition(angle: SIUnit<Radian>) {
//        angleMotor.encoder.resetPosition(angle)
//    }
//
//    override var brakeMode: Boolean
//        get() = angleMotor.brakeMode
//        set(v) {
//            angleMotor.brakeMode = brakeMode
//        }
//
//    override val brakeModeSwitch: Boolean
//        get() = _brakeModeSwitch.get()
//
//    fun simulationPeriodic() {
//
//        if (angleMotor.encoder.position < 90.degrees || angleMotor.encoder.position > 300.degrees) {
//            angleMotor.setVoltage(0.0.volts)
//            angleMotor.encoder.resetPosition(angleMotor.encoder.position.inDegrees().coerceIn(90.0, 300.0).degrees)
//        }
//        revMotorSim.run()
//    }
//
//}
