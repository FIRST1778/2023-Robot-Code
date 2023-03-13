package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.controller.PIDController
import org.frc1778.subsystems.Drive
import org.frc1778.Robot
import org.ghrobotics.lib.commands.FalconCommand
import com.ctre.phoenix.ErrorCode as PhoenixErrorCode

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        const val PROPORTIONAL: Double = 0.1 // (m/sec)/rad; extra slow to be careful
        const val INTEGRAL: Double = 0.0
        const val DERIVATIVE: Double = 0.0
        const val ERROR_TOLERANCE: Double = 0.2 // rad
        const val ERROR_DERIVATIVE_TOLERANCE: Double = 0.4 // rad/sec
    }

    // Note that the PIDController defaults to being called every 20ms.  So we must
    // calculate a velocity ONCE per tick: not more, not less.
    var pid: PIDController? = null
    override fun initialize() {
        if (Math.abs(Drive.robotPosition.rotation.radians) > 0.1)
            DriverStation.reportWarning("Smart balancing assumes 0 yaw", false)
        pid = PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE)
        pid!!.enableContinuousInput(0.0, 2.0*Math.PI)
        pid!!.setTolerance(ERROR_TOLERANCE, ERROR_DERIVATIVE_TOLERANCE)
    }

    override fun execute() {
        val pitch = Math.toRadians(Drive.pigeon.pitch)
        val velocity = pid!!.calculate(Math.sin(pitch), 0.0)
        Drive.swerveDrive(velocity, 0.0, 0.0, true)
    }

    override fun isFinished(): Boolean {
        return pid!!.atSetpoint()
    }
}