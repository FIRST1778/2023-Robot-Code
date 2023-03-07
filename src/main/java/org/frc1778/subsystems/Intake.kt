package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.PneumaticsModuleType
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
//    var intakeMotor = CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless)
//    var intakeMotor2 = CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless)

    var hopperMotor = falconMAX(14, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        smartCurrentLimit = 20.amps
    }
    var intakeMotor = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
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
        intakeMotor.setVoltage(-intakeVoltage / 6)
        hopperMotor.setVoltage(-intakeVoltage * 0.75)
    }

    fun spit() {
        hopperMotor.setVoltage(intakeVoltage / 2)
        intakeMotor.setVoltage(intakeVoltage + 2.0.volts)
    }

    fun stop() {
        hopperMotor.setVoltage(0.0.volts)
        intakeMotor.setVoltage(0.0.volts)
    }
}