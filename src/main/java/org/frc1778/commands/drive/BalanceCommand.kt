package org.frc1778.commands.drive

import edu.wpi.first.math.controller.PIDController
import org.frc1778.commands.lights.BalanceAnimation
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Gyro
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.sin

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        const val PROPORTIONAL: Double = 2.0 // (m/sec)/m
        const val INTEGRAL: Double = 0.05
        const val DERIVATIVE: Double = 0.125
        const val ERROR_TOLERANCE: Double = 0.055
        const val ERROR_DERIVATIVE_TOLERANCE: Double = 0.4
    }

    val balanceAnimation = BalanceAnimation()

    // Note that the PIDController defaults to being called every 20ms.  So we must
    // calculate a velocity ONCE per tick: not more, not less.
    lateinit var pid: PIDController
    override fun initialize() {
        pid = PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE)
        pid.enableContinuousInput(0.0, 2.0*Math.PI)
        pid.setTolerance(ERROR_TOLERANCE, ERROR_DERIVATIVE_TOLERANCE)
        balanceAnimation.initialize()
    }

    override fun execute() {
        val inclination = Gyro.boardInclination()
        val velocity = pid.calculate(sin(inclination), 0.0)
        Drive.swerveDrive(-velocity, 0.0, 0.0, true)
        balanceAnimation.execute()
    }

    override fun end(interrupted: Boolean) {
        DriveBrakeCommand().schedule()
        balanceAnimation.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return pid.atSetpoint()
    }
}