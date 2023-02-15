package org.frc1778.commands

import org.frc1778.Constants
import org.frc1778.Controls
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import kotlin.math.PI

class ArmTrapezoidCommand(endPosition : SIUnit<Radian>) : FalconCommand(Arm) {
    companion object {
        // TODO: made up these numbers
        const val MAX_VEL = 2.0  // rad/sec

        const val MAX_ACCEL = 1.5  // rad/sec^2

        const val START_VEL = 0.0   // rad/sec

        const val END_POS = PI/2.0  // rad
        const val END_VEL = 0.0     // rad/sec
    }


    var profile: TrapezoidProfile? = null
    var timer = Timer()

    override fun initialize() {
        timer.reset()
        timer.start()

        var startPosition: SIUnit<Radian> = Arm.getCurrentAngle()


        val constraints = TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        val startState = TrapezoidProfile.State(startPosition.value, START_VEL)
        val endState = TrapezoidProfile.State(END_POS, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
        Robot.dataLogger.add("Time") {timer.get()}
        Robot.dataLogger.add("Position") {Math.toDegrees(profile!!.calculate(timer.get()).position)}

    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        Arm.setAngle(state.position.radians)
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}