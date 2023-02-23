package org.frc1778.commands

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Robot
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.PI

class ExtensionCommand(endPosition : SIUnit<Meter>) : FalconCommand(Arm){
    companion object {
        // TODO: made up these numbers
        const val MAX_VEL = 0.45  // m/s

        const val MAX_ACCEL = 1.0  // m/s^2

        const val START_VEL = 0.0   // rad/sec

        const val END_POS = 0.381   // m
        const val END_VEL = 0.0     // rad/sec
    }


    var profile: TrapezoidProfile? = null
    var timer = Timer()

    override fun initialize() {
        timer.reset()
        timer.start()

        var startPosition: SIUnit<Meter> = Arm.getCurrentExtension()


        val constraints = TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        val startState = TrapezoidProfile.State(startPosition.value, START_VEL)
        val endState = TrapezoidProfile.State(END_POS, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
        Robot.dataLogger.add("Time") {timer.get()}
        Robot.dataLogger.add("Extension Position") {profile!!.calculate(timer.get()).position}

    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        Arm.setExtension(state.position.meters)
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}