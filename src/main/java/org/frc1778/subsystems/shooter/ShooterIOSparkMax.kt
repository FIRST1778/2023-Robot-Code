package org.frc1778.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX

class ShooterIOSparkMax: ShooterIO {

    private val nativeModel: NativeUnitRotationModel = NativeUnitRotationModel(42.nativeUnits)
    private val parentShooterMotor = falconMAX(11, CANSparkMaxLowLevel.MotorType.kBrushless, nativeModel) {
        outputInverted = false
        brakeMode = true

    }

    private val childShooterMotor = falconMAX(12, CANSparkMaxLowLevel.MotorType.kBrushless, nativeModel) {
        brakeMode = true
        outputInverted = false
        canSparkMax.follow(
            parentShooterMotor.canSparkMax,
            true
        )
    }

    private var loaded = false

    //TODO Get DIO
    private val limitSwitch = DigitalInput(2)

    override fun updateInputs(inputs: ShooterIOInputs) {
        if(!limitSwitch.get()) {
            inputs.shooterLoaded = true
        }
        inputs.shooterVoltage = parentShooterMotor.voltageOutput.value
        inputs.limitSwitchReading = limitSwitch.get()
    }

    override fun setVoltage(voltage: SIUnit<Volt>) {
        parentShooterMotor.setVoltage(voltage)
    }

    override fun setLoaded(loaded: Boolean) {
        this.loaded = loaded
    }

    override fun stop() {
        parentShooterMotor.setVoltage(0.0.volts)
    }

}