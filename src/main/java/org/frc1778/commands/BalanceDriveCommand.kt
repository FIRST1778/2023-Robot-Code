package org.frc1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Drive
import org.frc1778.Robot
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.math.geometry.Translation2D

class BalanceDriveCommand: FalconCommand(Drive) {
    private const val VELOCITY: Double = 0.5 // m/sec
    private const val MARGIN: Double = 0.2 // m
    private val xs = doubleArrayOf(5, 2) // m (needs to be adjusted to use alliance)
    private var stage = 0

    private var aprilTagsWereEnabled = false
    override fun initialize() {
        aprilTagsWereEnabled = Drive.aprilTagsEnabled
        Drive.aprilTagsEnabled = false
        stage = 0
    }

    override fun execute() {
        if (stage == xs.length)
            return
        val dx = xs[stage] - Drive.robotPosition.translation.getX()
        if (Math.abs(dx) < MARGIN) {
            stage++
        } else {
            Drive.swerveDrive(Math.signum(dx) * VELOCITY)
        }
    }

    override fun isFinished(): Boolean {
        return stage == xs.length
    }

    override fun end() {
        Drive.aprilTagsEnabled = aprilTagsWereEnabled
    }
}