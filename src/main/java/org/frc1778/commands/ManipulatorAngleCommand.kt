package org.frc1778.commands

import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians

class ManipulatorAngleCommand(var endPosition : SIUnit<Radian>) : FalconCommand(Manipulator) {
//    companion object {
//        const val END_VEL = 0.0     // rad/sec
//    }
//
//    var profile: TrapezoidProfile? = null
//    var timer = Timer()
//    val maxAcceleration : Double = 0.25
//    val maxVelocity: Double = 0.25
//    override fun initialize() {
//        val endPos = endPosition
//
//        timer.reset()
//        timer.start()
//
//        var startPosition: SIUnit<Radian> = Manipulator.getCurrentAngle()
//
//        val constraints = TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
//        val startState = TrapezoidProfile.State(startPosition.value, Manipulator.getDesiredAngleVelocity())
//        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
//        profile = TrapezoidProfile(constraints, endState, startState)
//    }
//
//    override fun execute() {
//        val state = profile!!.calculate(timer.get())
//        Manipulator.setDesiredAngleVelocity(state.velocity)
//        Manipulator.setDesiredAngle(state.position.radians)
//    }
//
//    override fun cancel() {
//        super.cancel()
//        Manipulator.setDesiredAngleVelocity(0.0)
//    }
//    override fun isFinished(): Boolean {
//        return profile!!.isFinished(timer.get())
//    }
}