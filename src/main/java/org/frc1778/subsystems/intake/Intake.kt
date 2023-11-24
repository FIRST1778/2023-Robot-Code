package org.frc1778.subsystems.intake

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.littletonrobotics.junction.Logger

object Intake : FalconSubsystem() {

    private val io = IntakeIOSparkMax()
    private val intakeInputs = IntakeInputsAutoLogged()

    private var intakeVoltage = 4.5.volts

    fun cubeStored() : Boolean {
        return io.cubeStored()
    }

    fun suck() {
        io.setVoltage(intakeVoltage + 2.0.volts)
    }
    fun extend(){
        io.extend()
    }
    fun retract(){
        io.retract()
    }

    fun spit() {
        io.setVoltage(-intakeVoltage * 1.75)
    }

    fun stop() {
        io.setVoltage(0.0.volts)
    }
    fun lineBreakOverrideToggle(){
        io.toggleLineBreakOverride()
    }

    override fun periodic() {
        io.updateInputs(intakeInputs)
        Logger.processInputs("Intake Inputs", intakeInputs)
    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addBooleanProperty("Line Break", ::cubeStored, {})
    }
    init {
        Shuffleboard.getTab("Intake").add(this)
    }
}
