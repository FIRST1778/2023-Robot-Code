package org.frc1778.commands.drive

import edu.wpi.first.math.controller.PIDController
import org.frc1778.commands.lights.BalanceAnimation
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Gyro
import org.ghrobotics.lib.commands.FalconCommand

class BalanceCommand: FalconCommand(Drive) {
    private object Config {
        const val proportional: Double = 2.0 // (m/sec)/m
        const val derivative: Double = 0.125
        const val error_tolerance: Double = 0.055
        const val derivative_tolerance: Double = 0.4
        const val backoff: Double = 0.4
    }

    val balanceAnimation = BalanceAnimation()

    var backoff = Double.NaN
    var previousSign = Double.NaN
    lateinit var pid: PIDController

    override fun initialize() {
        backoff = 1.0
        previousSign = 0.0
        balanceAnimation.initialize()
        pid = PIDController(Config.proportional, 0.0, Config.derivative)
        pid.enableContinuousInput(0.0, 2.0*Math.PI)
        pid.setTolerance(Config.error_tolerance, Config.derivative_tolerance)
    }

    override fun execute() {
        val inclination = Math.sin(Gyro.boardInclination())
        if (Math.signum(inclination) * previousSign == -1.0)
            backoff *= Config.backoff
        previousSign = Math.signum(inclination)

        val xvel = backoff * pid.calculate(inclination, /* setpoint */ 0.0)
        Drive.swerveDrive(xvel, 0.0, 0.0, true)

        balanceAnimation.execute()
    }

    override fun end(interrupted: Boolean) {
        balanceAnimation.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return pid.atSetpoint()
    }
}
