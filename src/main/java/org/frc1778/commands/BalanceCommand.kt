package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.frc1778.Constants
import org.frc1778.subsystems.Drive
import org.frc1778.Robot
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.withSign
import com.ctre.phoenix.ErrorCode as PhoenixErrorCode

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        const val PROPORTIONAL: Double = 2.125 // (m/sec)/rad; extra slow to be careful
        const val INTEGRAL: Double = 0.0
        const val DERIVATIVE: Double = 0.185
        const val ERROR_TOLERANCE: Double = 0.055 // rad
        const val ERROR_DERIVATIVE_TOLERANCE: Double = 0.4 // rad/sec
    }

    var pitch = 0.0
    var velocity = 0.0

    // Note that the PIDController defaults to being called every 20ms.  So we must
    // calculate a velocity ONCE per tick: not more, not less.
    lateinit var pid: PIDController
    override fun initialize() {
        if (Math.abs(Drive.robotPosition.rotation.radians) > 0.1)
            DriverStation.reportWarning("Smart balancing assumes 0 yaw", false)
        pid = PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE)
        pid.enableContinuousInput(0.0, 2.0*Math.PI)
        pid.setTolerance(ERROR_TOLERANCE, ERROR_DERIVATIVE_TOLERANCE)

    }



    override fun execute() {
        pitch = Math.toRadians(Drive.pigeon.pitch)
        velocity = pid.calculate(sin(pitch), 0.0)
        velocity = -velocity * Robot.driveInversion
        if (cos(Drive.robotPosition.rotation.radians) < 0)
            velocity = -velocity
        Drive.swerveDrive(velocity, 0.0, 0.0, true)
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder?.addDoubleProperty("Pitch", {pitch},  {})
        builder?.addDoubleProperty("Velocity", {velocity}, {})
    }

    override fun end(interrupted: Boolean) {
        DriveBrakeCommand().schedule()
    }

    override fun isFinished(): Boolean {
        return pid.atSetpoint()
    }
}