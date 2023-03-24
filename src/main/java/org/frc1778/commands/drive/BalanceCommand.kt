package org.frc1778.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.atan
import kotlin.math.sin

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        const val PROPORTIONAL: Double = 1.95 // (m/sec)/m
        const val INTEGRAL: Double = 0.05
        const val DERIVATIVE: Double = 0.17
        const val ERROR_TOLERANCE: Double = 0.055
        const val ERROR_DERIVATIVE_TOLERANCE: Double = 0.4

        
    }

    // Note that the PIDController defaults to being called every 20ms.  So we must
    // calculate a velocity ONCE per tick: not more, not less.
    lateinit var pid: PIDController
    override fun initialize() {
        pid = PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE)
        pid.enableContinuousInput(0.0, 2.0*Math.PI)
        pid.setTolerance(ERROR_TOLERANCE, ERROR_DERIVATIVE_TOLERANCE)
    }

    override fun execute() {
        val inclination = Drive.boardInclination()
        val velocity = pid.calculate(sin(inclination), 0.0)
        Drive.swerveDrive(-velocity, 0.0, 0.0, true)
    }

    override fun end(interrupted: Boolean) {
        DriveBrakeCommand().schedule()
    }

    override fun isFinished(): Boolean {
        return pid.atSetpoint()
    }
}