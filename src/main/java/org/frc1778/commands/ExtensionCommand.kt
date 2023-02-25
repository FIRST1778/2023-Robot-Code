package org.frc1778.commands

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.meters

class ExtensionCommand(endPosition : SIUnit<Meter>, maxAcceleration : Double = 1.0, maxVelocity : Double = 0.45) : FalconCommand(Arm){
    companion object {
        const val START_VEL = 0.0   // rad/sec
        const val END_VEL = 0.0     // rad/sec
    }


    private var profile: TrapezoidProfile? = null
    private var timer = Timer()

    private var endPos = endPosition
    private var maxAccel = maxAcceleration
    private var maxVel = maxVelocity
    override fun initialize() {
        timer.reset()
        timer.start()

        var startPosition: SIUnit<Meter> = Arm.getCurrentExtension()


        val constraints = TrapezoidProfile.Constraints(maxVel, maxAccel)
        val startState = TrapezoidProfile.State(startPosition.value, START_VEL)
        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        Arm.setExtensionVelocity(state.velocity)
        Arm.setExtension(state.position.meters)
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}