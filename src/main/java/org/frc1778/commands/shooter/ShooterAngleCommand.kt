package org.frc1778.commands.shooter

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Level
import org.frc1778.lib.GameObject
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians

class ShooterAngleCommand(val scoringLevel : Level) : FalconCommand(Shooter) {
    companion object {
        const val END_VEL = 0.0     // rad/sec
    }

    lateinit var profile: TrapezoidProfile
    var timer = Timer()
    val maxAcceleration : Double = 3.5 // rad/sec
    val maxVelocity: Double = 2.125
    override fun initialize() {
        val endPos = scoringLevel.shooterPosition
        timer.reset()
        timer.start()

        var startPosition: SIUnit<Radian> = Shooter.getCurrentAngle()

        val constraints = TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
        val startState = TrapezoidProfile.State(startPosition.value, Shooter.getDesiredAngleVelocity())
        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
    }

    override fun execute() {
        val state = profile.calculate(timer.get())
        Shooter.setDesiredAngleVelocity(state.velocity)
        Shooter.setDesiredAngle(state.position.radians)
    }

    override fun cancel() {
        super.cancel()
        Shooter.setDesiredAngleVelocity(0.0)
    }
    override fun isFinished(): Boolean {
        return profile.isFinished(timer.get())
    }
    override fun end(interrupted: Boolean){
        Shooter.setScoringLevel(scoringLevel)
    }
}