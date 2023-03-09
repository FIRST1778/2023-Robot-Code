package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Drive
import org.frc1778.Robot
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.abs
import kotlin.math.sign

class BalanceDriveCommand : FalconCommand(Drive) {

    private var stage = 0

    private var aprilTagsWereEnabled = false
    override fun initialize() {
        aprilTagsWereEnabled = Drive.aprilTagsEnabled
        Drive.aprilTagsEnabled = false
        stage = 0
    }

    override fun execute() {
        if (stage == xs.size) return
        val dx = xs[stage] - Drive.robotPosition.translation.x
        if (abs(dx) < MARGIN) {
            stage++
        } else {
            Drive.swerveDrive(sign(dx) * VELOCITY, 0.0, 0.0)
        }
    }

    override fun isFinished(): Boolean {
        return stage == xs.size
    }

    override fun end(interrupted: Boolean) {
        Drive.aprilTagsEnabled = aprilTagsWereEnabled
    }

    companion object {
        private const val MARGIN: Double = 0.2 // m
        private const val VELOCITY: Double = 0.5 // m/sec
        private val xs = doubleArrayOf(5.0, 2.0) // m (needs to be adjusted to use alliance)
    }
}