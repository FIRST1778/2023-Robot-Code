package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
    var intakeMotor = CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless)


    val intakeSol = FalconDoubleSolenoid(
        1,
        0,
        PneumaticsModuleType.REVPH,
        30
    )

    fun open(){
        intakeSol.state = FalconSolenoid.State.Forward
    }
    fun retract(){
        intakeSol.state = FalconSolenoid.State.Reverse
    }
    fun suck(){
        intakeMotor.setVoltage(-5.0)
    }
    fun spit(){
        intakeMotor.setVoltage(5.0)
    }
    fun stop(){
        intakeMotor.setVoltage(0.0)
    }
}