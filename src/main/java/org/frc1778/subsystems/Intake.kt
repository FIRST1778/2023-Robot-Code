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

    var intakeMotor = falconMAX(15, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(46.nativeUnits)) {
        brakeMode = true
        smartCurrentLimit = 20.amps
    }

    var intakeVoltage = 5.0.volts


    fun suck() {
        if (!Manipulator.lineBreak.get()){
            intakeMotor.setVoltage(-intakeVoltage)
        }
    }

    fun spit() {
        intakeMotor.setVoltage(intakeVoltage)
    }

    fun stop() {
        intakeMotor.setVoltage(0.0.volts)
    }
}