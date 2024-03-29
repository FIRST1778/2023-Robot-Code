package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.frc1778.Level
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
    //TODO Get linebreak channel
    val lineBreak: DigitalInput = DigitalInput(1)


    var beltMotor = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        outputInverted = true
    }
    var wheelMotor = falconMAX(14, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)){
        brakeMode = true
        outputInverted = true
    }
    val solenoid = FalconDoubleSolenoid(
        0,
        1,
        PneumaticsModuleType.REVPH,
        30
    )
    var lineBreakOverride : Boolean = false

    private var intakeVoltage = 10.volts

    private fun setMotorVoltage(voltage : SIUnit<Volt>){
        beltMotor.setVoltage(voltage/3)
        wheelMotor.setVoltage(voltage/5)
    }
    fun cubeStored() : Boolean {
        return if (!lineBreakOverride) {
            !lineBreak.get()
        }else{
            false
        }
    }

    fun suck() {
//        if (Manipulator.lineBreak.get()){
//            intakeMotor.setVoltage(-intakeVoltage)
//        }
        setMotorVoltage(intakeVoltage + 2.0.volts)
    }
    fun extend(){
        if (Wrist.encoder.absolutePosition > 180.0.degrees) {
            Wrist.setNextLevel(Level.None)
        }
        if (cubeStored()) {
            retract()
        } else {
            solenoid.state = FalconSolenoid.State.Forward
        }
    }
    fun retract(){
        solenoid.state = FalconSolenoid.State.Reverse
    }

    fun spit() {
        setMotorVoltage(-intakeVoltage * 1.75)
    }

    fun stop() {
        setMotorVoltage(0.0.volts)
    }
    fun lineBreakOverrideToggle(){
        lineBreakOverride = !lineBreakOverride
    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addBooleanProperty("Line Break", { lineBreak.get()}, {})
    }
    init {
        Shuffleboard.getTab("Intake").add(this)
    }
}
