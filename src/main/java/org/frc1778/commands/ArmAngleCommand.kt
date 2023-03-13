package org.frc1778.commands

import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians

class ArmAngleCommand(endPosition : SIUnit<Radian>) : FalconCommand(Arm, Manipulator) {


//    companion object {
//        const val END_VEL = 0.0     // rad/sec
//    }
//
//    var profile: TrapezoidProfile? = null
//    var timer = Timer()
//    val maxAcceleration : Double = 0.4
//    val maxVelocity: Double = 0.4
//    var endPos = endPosition
//    override fun initialize() {
//        timer.reset()
//        timer.start()
//
//        var startPosition: SIUnit<Radian> = Arm.getCurrentAngle()
//
//        val constraints = TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
//        val startState = TrapezoidProfile.State(startPosition.value, Arm.getDesiredAngleVelocity())
//        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
//        profile = TrapezoidProfile(constraints, endState, startState)
//    }
//
//    override fun execute() {
//        val state = profile!!.calculate(timer.get())
//        Arm.setDesiredAngleVelocity(state.velocity)
//        Arm.setDesiredAngle(state.position.radians)
//    }
//
//    override fun cancel() {
//        super.cancel()
//        Arm.setDesiredAngleVelocity(0.0)
//    }
//    override fun isFinished(): Boolean {
//        return profile!!.isFinished(timer.get())
//    }
}