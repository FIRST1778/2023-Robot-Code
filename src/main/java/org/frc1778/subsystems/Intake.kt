package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
    //TODO Get linebreak channel
    val lineBreak: DigitalInput = DigitalInput(3)

    var beltMotor = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        outputInverted = true
    }
    var wheelMotor = falconMAX(14, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)){
        brakeMode = true
        outputInverted = true
    }
    val solenoid = FalconDoubleSolenoid(
        5,
        3,
        PneumaticsModuleType.REVPH,
        30
    )

    private var intakeVoltage = 5.0.volts
    fun setMotorVoltage(voltage : SIUnit<Volt>){
        beltMotor.setVoltage(voltage)
        wheelMotor.setVoltage(voltage/5)
    }

    fun suck() {
//        if (Manipulator.lineBreak.get()){
//            intakeMotor.setVoltage(-intakeVoltage)
//        }
        setMotorVoltage(intakeVoltage + 2.0.volts)
        solenoid.state = FalconSolenoid.State.Forward
    }
    fun extend(){
        solenoid.state = FalconSolenoid.State.Forward
    }
    fun retract(){
        solenoid.state = FalconSolenoid.State.Reverse
    }

    fun spit() {
        setMotorVoltage(-intakeVoltage * 1.25)
    }

    fun stop() {
        setMotorVoltage(0.0.volts)
    }
}