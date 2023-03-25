package org.frc1778.commands.shooter

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.Level
import org.frc1778.Robot
import org.frc1778.lib.GameObject
import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import kotlin.math.PI
import kotlin.math.round

class ShooterAngleCommand(val scoringLevel : Level) : FalconCommand(Shooter) {
    companion object {
        const val END_VEL = 0.0     // rad/sec
    }

    lateinit var profile: TrapezoidProfile
    var timer = Timer()
    val maxAcceleration : Double = 3.5 // rad/sec
    val maxVelocity: Double = 2.125
    override fun initialize() {
        val directionTowardsGrid = when (Robot.alliance) {
            DriverStation.Alliance.Red -> 0.0
            else -> PI
        }
        val endPos = if (directionTowardsGrid == PI * round(Gyro.odometryYaw() / PI)) {
            scoringLevel.frontShooterPosition
        }else{
            scoringLevel.rearShooterPosition
        }
        if(endPos > 180.0.degrees){
            Intake.retract()
        }
        timer.reset()
        timer.start()
        Shooter.setNextLevel(scoringLevel)
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