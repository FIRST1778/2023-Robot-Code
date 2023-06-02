package org.frc1778.subsystems.shooter

import edu.wpi.first.util.sendable.Sendable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.littletonrobotics.junction.Logger

object Shooter : FalconSubsystem(), Sendable {

    private var io: ShooterIO = ShooterIOSparkMax()
    private val inputs = ShooterIOInputsAutoLogged()

    var cubeStored: Boolean
        get() = inputs.shooterLoaded
        set(v) {
            io.setLoaded(v)
        }
    fun shoot(voltage: SIUnit<Volt>) {
        io.setVoltage(voltage)
    }

    //TODO Get best intake speed
    fun suck() {
        io.setVoltage((-1.5).volts)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Shooter Inputs", inputs)
    }

//    override fun lateInit() {
//        Constants.ShooterConstants.shooterTab.add(
//            this
//        ).withSize(3, 4)
//
//        Constants.ShooterConstants.shooterTab.add(
//            "Limit Switch",
//            limitSwitch
//        ).withSize(2, 2)
//
//    }

//    override fun initSendable(builder: SendableBuilder?) {
//        super.initSendable(builder)
//        builder!!.addDoubleProperty("Shooter Voltage", {
//            parentShooterMotor.voltageOutput.value
//        }, {})
//        builder.addBooleanProperty("Shooter Loaded", { cubeStored }, {})
//    }


    fun stopWheels() {
        io.stop()
    }

    fun spit() {
        io.setVoltage(0.75.volts)
    }

}

