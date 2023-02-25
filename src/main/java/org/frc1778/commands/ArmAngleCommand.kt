package org.frc1778.commands

import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians

class ArmAngleCommand(endPosition : SIUnit<Radian>, maxAcceleration : Double = 1.5, maxVelocity: Double = 2.0) : FalconCommand(Arm) {
    companion object {
        const val START_VEL = 0.0   // rad/sec
        const val END_VEL = 0.0     // rad/sec
    }

    var profile: TrapezoidProfile? = null
    var timer = Timer()

    var maxAccel = maxAcceleration
    var endPos = endPosition
    var maxVel = maxVelocity
    override fun initialize() {
        timer.reset()
        timer.start()

        var startPosition: SIUnit<Radian> = Arm.getCurrentAngle()


        val constraints = TrapezoidProfile.Constraints(maxVel, maxAccel)
        val startState = TrapezoidProfile.State(startPosition.value, START_VEL)
        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        Arm.setAngleVelocity(state.velocity)
        Arm.setAngle(state.position.radians)
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}