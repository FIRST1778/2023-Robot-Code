package org.frc1778.commands

import org.frc1778.Constants
import org.frc1778.Controls
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.radians
import kotlin.math.PI

class ArmTrapezoidCommand : FalconCommand(Arm) {
    companion object {
        // TODO: made up these numbers
        const val MAX_VEL   = 2.0   // rad/sec
        const val MAX_ACCEL = 10.0  // rad/sec^2

        const val START_POS = 0.0   // rad
        const val START_VEL = 0.0   // rad/sec

        const val END_POS = PI/2.0  // rad
        const val END_VEL = 0.0     // rad/sec
    }

    val constraints = TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
    val startState = TrapezoidProfile.State(START_POS, START_VEL)
    val endState = TrapezoidProfile.State(END_POS, END_VEL)
    val profile = TrapezoidProfile(constraints, startState, endState)

    var timer = Timer()

    override fun initialize() {
        timer.reset()
    }

    override fun execute() {
        val state = profile.calculate(timer.get())
        Arm.setAngle(state.position.radians)
    }

    override fun isFinished(): Boolean {
        return profile.isFinished(timer.get())
    }
}