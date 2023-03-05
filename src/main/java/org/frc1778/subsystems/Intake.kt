package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
//    var intakeMotor = CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless)
//    var intakeMotor2 = CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless)

    var intakeMotor = falconMAX(14, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        smartCurrentLimit = 20.amps
    }
    var intakeMotor2 = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        smartCurrentLimit = 20.amps
    }


    var intakeVoltage = 8.0.volts


    val intakeSol = FalconDoubleSolenoid(
        3, 5, PneumaticsModuleType.REVPH, 30
    )

    fun extend() {
        intakeSol.state = FalconSolenoid.State.Forward
    }

    fun toggle() {

    }

    fun retract() {
        intakeSol.state = FalconSolenoid.State.Reverse
    }

    fun suck() {
        if (intakeSol.state == FalconSolenoid.State.Reverse || intakeSol.state == FalconSolenoid.State.Off) {
            extend()
        }
        intakeMotor2.setVoltage(-intakeVoltage / 6)
        intakeMotor.setVoltage(-intakeVoltage * 0.75)
    }

    fun spit() {
        intakeMotor.setVoltage(intakeVoltage / 2)
        intakeMotor2.setVoltage(intakeVoltage / 2)
    }

    fun stop() {
        intakeMotor.setVoltage(0.0.volts)
        intakeMotor2.setVoltage(0.0.volts)
    }
}