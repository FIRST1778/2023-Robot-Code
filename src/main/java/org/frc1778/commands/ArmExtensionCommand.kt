package org.frc1778.commands

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.meters

class ArmExtensionCommand(endPosition : SIUnit<Meter>) : FalconCommand(Arm){
    companion object {
        const val START_VEL = 0.0   // rad/sec
        const val END_VEL = 0.0     // rad/sec
    }

    private val maxAcceleration : Double = 0.75
    private val maxVelocity : Double = 1.10
    private var profile: TrapezoidProfile? = null
    private var timer = Timer()

    private var endPos = endPosition
    private var maxAccel = maxAcceleration
    private var maxVel = maxVelocity
    override fun initialize() {
        timer.reset()
        timer.start()
//TODO add correct values
        var startPosition: SIUnit<Meter> = Arm.getCurrentExtension()


        val constraints = TrapezoidProfile.Constraints(maxVel, maxAccel)
        val startState = TrapezoidProfile.State(startPosition.value, START_VEL)
        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        Arm.setDesiredExtensionVelocity(state.velocity)
        Arm.desiredExtension = state.position.meters
    }

    override fun end(interrupted: Boolean) {
        DriverStation.reportError("Extension Finished", false)
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}