package org.frc1778.subsystems.intake

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PneumaticsModuleType
import org.frc1778.Level
import org.frc1778.subsystems.wrist.Wrist
import org.ghrobotics.lib.junction.AutoLog
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

interface IntakeIO {

    fun updateInputs(inputs: IntakeInputs)

    fun extend()

    fun retract()

    fun cubeStored(): Boolean

    fun setVoltage(voltage: SIUnit<Volt>)

    fun setBeltVoltage(voltage: SIUnit<Volt>)

    fun setWheelVoltage(voltage: SIUnit<Volt>)

    fun setSolenoidState(state: FalconSolenoid.State)

    fun toggleLineBreakOverride()
}

@AutoLog
open class IntakeInputs() {
    var solenoidState: String = ""

    var beltVoltage: SIUnit<Volt> = 0.volts
    var beltCurrent: SIUnit<Ampere> = 0.amps

    var wheelVoltage: SIUnit<Volt> = 0.volts
    var wheelCurrent: SIUnit<Ampere> = 0.amps

    var lineBreak: Boolean = false
    var lineBreakOverride: Boolean = false

}

class IntakeIOSparkMax: IntakeIO {
    private val lineBreak: DigitalInput = DigitalInput(1)


    private var beltMotor = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        outputInverted = true
    }
    private var wheelMotor = falconMAX(14, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)){
        brakeMode = true
        outputInverted = true
    }
    private val solenoid = FalconDoubleSolenoid(
        0,
        1,
        PneumaticsModuleType.REVPH,
        30
    )
    private var lineBreakOverride : Boolean = false

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.run {
            solenoidState = solenoid.state.name

            beltVoltage = beltMotor.voltageOutput
            beltCurrent = beltMotor.drawnCurrent

            wheelVoltage = wheelMotor.voltageOutput
            wheelCurrent = wheelMotor.drawnCurrent

            lineBreak = this@IntakeIOSparkMax.lineBreak.get()
            lineBreakOverride= this@IntakeIOSparkMax.lineBreakOverride
        }
    }

    override fun extend() {
        if (Wrist.getCurrentAngle() > 180.0.degrees) {
            Wrist.setNextLevel(Level.None)
        }
        if (Intake.cubeStored()) {
            Intake.retract()
        } else {
            solenoid.state = FalconSolenoid.State.Forward
        }
    }

    override fun retract() {
        solenoid.state = FalconSolenoid.State.Reverse
    }

    override fun cubeStored(): Boolean {
        return if (!lineBreakOverride) {
            !lineBreak.get()
        }else{
            false
        }
    }

    override fun setVoltage(voltage: SIUnit<Volt>) {
        setBeltVoltage(voltage)
        setWheelVoltage(voltage/5)
    }

    override fun setBeltVoltage(voltage: SIUnit<Volt>) {
        beltMotor.setVoltage(voltage)
    }

    override fun setWheelVoltage(voltage: SIUnit<Volt>) {
       wheelMotor.setVoltage(voltage)
    }

    override fun setSolenoidState(state: FalconSolenoid.State) {
        solenoid.state = state
    }

    override fun toggleLineBreakOverride() {
        lineBreakOverride = !lineBreakOverride
    }

}