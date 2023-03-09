package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.frc1778.subsystems.Drive
import org.frc1778.Robot
import org.ghrobotics.lib.commands.FalconCommand
import com.ctre.phoenix.ErrorCode as PhoenixErrorCode

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        private const val MARGIN: Double = 0.2  // rad
        private const val VELOCITY: Double = 0.2  // m/s
        private const val DESIRED_LEVEL_COUNT: Int = 25  // 500 ms
    }

    var levelCount = 0

    override fun initialize() {
        levelCount = 0
    }

    private fun level(): Boolean {
        val pitch = Drive.pigeon.pitch
        val roll = Drive.pigeon.roll
        val inclination = Math.atan(Math.hypot(Math.tan(pitch), Math.tan(roll)))

        return Math.abs(inclination) < MARGIN
    }

    override fun execute() {
        val pitch = Drive.pigeon.pitch
        val yaw = Drive.robotPosition.rotation.radians
        var x = Math.cos(yaw) * Math.cos(pitch)
        val z = Math.sin(pitch)
        val dx = Math.signum(x) * Math.signum(z)
        Drive.swerveDrive(dx * VELOCITY, 0.0, 0.0, true)
    }

    override fun isFinished(): Boolean {
        if (level()) {
            return ++levelCount == DESIRED_LEVEL_COUNT
        } else {
            levelCount = 0
            return false
        }
    }
}