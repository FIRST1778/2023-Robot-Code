package org.frc1778.commands.drive

import org.frc1778.commands.lights.BalanceAnimation
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Gyro
import org.ghrobotics.lib.commands.FalconCommand

class BalanceCommand: FalconCommand(Drive) {
    // Balance controller.  Velocity is proportional to sine of angle
    // error.  Each time the balance board tips over, we decrease our
    // velocity by a constant factor so that we stabilize in the center.
    //
    // The old, unreliable autonomous balance, which had no exponential
    // backoff and used a PID controller, is in OldBalanceCommand.kt.
    // To switch back to it, just rename the class back to
    // BalanceCommand (the interface is the same).

    private object Constants {
        const val proportional: Double = 2.0  // (m/s) / m
        const val derivative: Double = 0.125  // (m/s) / (m/s)
        const val backoff: Double = 0.4
        const val tolerance: Double = 0.055  // m
        const val errorTolerance: Double = 0.4
    }

    val balanceAnimation = BalanceAnimation()

    var sign = Double.NaN
    var multiplier = Double.NAN

    override fun initialize() {
        var multiplier = 1.0
        sign = 0.0
        balanceAnimation.initialize()
    }

    override fun execute() {
        val inclination = Math.sin(Gyro.boardInclination())
        if (Math.signum(inclination) * sign == -1.0)
            factor *= Constants.backoff
        sign = Math.signum(inclination)
        Drive.swerveDrive(inclination * factor, 0.0, 0.0, true)
        balanceAnimation.execute()
    }

    override fun end(interrupted: Boolean) {
        balanceAnimation.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return Math.abs(Math.sin(Gyro.boardInclination())) < Constants.tolerance
    }
}
