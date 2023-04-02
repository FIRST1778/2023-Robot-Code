package org.frc1778.commands.shooter

import com.github.ajalt.colormath.model.RGB
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Level
import org.frc1778.animation.BlinkAnimation
import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.frc1778.subsystems.Wrist
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians

class ShooterAngleCommand(val scoringLevel: Level) : FalconCommand(Wrist) {

    companion object {
        const val END_VEL = 0.0     // rad/sec
    }

    lateinit var profile: TrapezoidProfile
    var timer = Timer()
    val maxAcceleration: Double = 9.0 // rad/sec
    val maxVelocity: Double = 5.0
    var endPos: SIUnit<Radian>? = null

    override fun initialize() {
        val endPos = if (!Shooter.cubeStored) {
            Level.None.frontShooterPosition //Can't angle if no cube
        } else if (Gyro.direction180() == Gyro.directionTowardsGrid()) {
            scoringLevel.frontShooterPosition
        } else {
            scoringLevel.rearShooterPosition
        }

        if (endPos > 180.0.degrees) {
            Intake.retract()
        }
        timer.reset()
        timer.start()
        Wrist.setNextLevel(scoringLevel)
        var startPosition: SIUnit<Radian> = Wrist.getCurrentAngle()

        val constraints = TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
        val startState = TrapezoidProfile.State(startPosition.value, Wrist.getDesiredAngleVelocity())
        val endState = TrapezoidProfile.State(endPos.value, END_VEL)
        profile = TrapezoidProfile(constraints, endState, startState)
        this.endPos = endPos
    }

    override fun execute() {
        val state = profile.calculate(timer.get())
        Wrist.setDesiredAngleVelocity(state.velocity)
        Wrist.setDesiredAngle(state.position.radians)
    }

    override fun cancel() {
        super.cancel()
        Wrist.setDesiredAngleVelocity(0.0)
        Wrist.setDesiredAngle(Wrist.getCurrentAngle())
        end(true)
    }

    override fun isFinished(): Boolean {
        return profile.isFinished(timer.get())
    }

    override fun end(interrupted: Boolean) {

        if (!interrupted) {
            Wrist.setScoringLevel(scoringLevel)
            Lights.setAnimation(
                BlinkAnimation(RGB.from255(0, 255, 0), RGB, 6, 6)
            )
        }
    }
}
