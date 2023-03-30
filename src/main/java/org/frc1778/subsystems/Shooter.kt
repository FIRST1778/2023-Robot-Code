package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import org.frc1778.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX

object Shooter : FalconSubsystem(), Sendable {


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

    //TODO Get DIO
    private val limitSwitch = DigitalInput(2)



    var cubeStored = false

    fun shoot(voltage : SIUnit<Volt>){
        parentShooterMotor.setVoltage(voltage)
    }
    //TODO Get best intake speed
    fun suck(){
       parentShooterMotor.setVoltage((-1.5).volts)
    }

    override fun periodic() {


        if(!limitSwitch.get()){
            cubeStored = true
        }
    }
    override fun lateInit() {
        Constants.ShooterConstants.shooterTab.add(
            this
        ).withSize(3, 4)

        Constants.ShooterConstants.shooterTab.add(
            "Limit Switch",
            limitSwitch
        ).withSize(2, 2)

    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addDoubleProperty("Shooter Voltage", {
            parentShooterMotor.voltageOutput.value
        }, {})
        builder.addBooleanProperty("Shooter Loaded", { cubeStored}, {})
    }

    fun stopWheels() {
        parentShooterMotor.setVoltage(0.0.volts)
    }

    fun spit() {
        parentShooterMotor.setVoltage(0.75.volts)
    }
}