package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
    var intakeMotor = CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless)
    var intakeMotor2 = CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless)

    var intakeVoltage = 5.0

    lateinit var initialState : FalconSolenoid.State

    val intakeSol = FalconDoubleSolenoid(
        1,
        0,
        PneumaticsModuleType.REVPH,
        30
    )

    fun extend(){
        intakeSol.state = FalconSolenoid.State.Forward
    }
    fun toggle(){

    }
    fun retract(){
        intakeSol.state = FalconSolenoid.State.Reverse
    }
    fun suck(){
        if(intakeSol.state == FalconSolenoid.State.Reverse){
            extend()
        }
        intakeMotor2.setVoltage(intakeVoltage)
        intakeMotor.setVoltage(-intakeVoltage)
    }
    fun spit(){
        intakeMotor.setVoltage(intakeVoltage)
        intakeMotor2.setVoltage(-intakeVoltage)
    }
    fun stop(){
        intakeMotor.setVoltage(0.0)
        intakeMotor2.setVoltage(0.0)
    }
}