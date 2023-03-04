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

    var intakeVoltage = 6.0


    val intakeSol = FalconDoubleSolenoid(
        3,
        5,
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
        if(intakeSol.state == FalconSolenoid.State.Reverse || intakeSol.state == FalconSolenoid.State.Off){
            extend()
        }
        intakeMotor2.setVoltage(-intakeVoltage/6)
        intakeMotor.setVoltage(-intakeVoltage * 0.75)
    }
    fun spit(){
        intakeMotor.setVoltage(intakeVoltage/2)
        intakeMotor2.setVoltage(intakeVoltage/4)
    }
    fun stop(){
        intakeMotor.setVoltage(0.0)
        intakeMotor2.setVoltage(0.0)
    }
}