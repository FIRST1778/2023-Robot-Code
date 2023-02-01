/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.frc1778.lib

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.ghrobotics.lib.mathematics.max
import kotlin.math.absoluteValue
import kotlin.math.withSign

/**
 * Helper class that contains all types of driving -- tank drive,
 * curvature drive, arcade drive.
 */
class FalconDriveHelper {
    private var quickStopAccumulator = 0.0

    /**
     * Drives the robot using arcade drive.
     *
     * @param linearPercent The percent for linear motion.
     * @param rotationPercent The percent for angular rotation.
     */
    fun arcadeDrive(
        linearPercent: Double, rotationPercent: Double
    ): Pair<Double, Double> {
        val maxInput = max(linearPercent.absoluteValue, rotationPercent.absoluteValue).withSign(linearPercent)

        val leftMotorOutput: Double
        val rightMotorOutput: Double

        if (linearPercent >= 0.0) {
            // First quadrant, else second quadrant
            if (rotationPercent >= 0.0) {
                leftMotorOutput = maxInput
                rightMotorOutput = linearPercent - rotationPercent
            } else {
                leftMotorOutput = linearPercent + rotationPercent
                rightMotorOutput = maxInput
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (rotationPercent >= 0.0) {
                leftMotorOutput = linearPercent + rotationPercent
                rightMotorOutput = maxInput
            } else {
                leftMotorOutput = maxInput
                rightMotorOutput = linearPercent - rotationPercent
            }
        }

        return Pair(leftMotorOutput, rightMotorOutput)
    }

    /**
     * Drives the robot using curvature drive.
     *
     * @param linearPercent The percent for linear motion.
     * @param curvaturePercent The percent for curvature of the robot.
     * @param isQuickTurn Whether to use arcade drive or not.
     */
    fun curvatureDrive(
        linearPercent: Double, curvaturePercent: Double, isQuickTurn: Boolean
    ): Pair<Double, Double> {
        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (linearPercent.absoluteValue < kQuickStopThreshold) {
                quickStopAccumulator =
                    (1 - kQuickStopAlpha) * quickStopAccumulator + kQuickStopAlpha * curvaturePercent.coerceIn(
                        -1.0,
                        1.0
                    ) * 2.0
            }
            overPower = true
            angularPower = curvaturePercent
        } else {
            overPower = false
            angularPower = linearPercent.absoluteValue * curvaturePercent - quickStopAccumulator

            when {
                quickStopAccumulator > 1 -> quickStopAccumulator -= 1.0
                quickStopAccumulator < -1 -> quickStopAccumulator += 1.0
                else -> quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = linearPercent + angularPower
        var rightMotorOutput = linearPercent - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }

                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }

                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }

                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = max(leftMotorOutput.absoluteValue, rightMotorOutput.absoluteValue)
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        return Pair(leftMotorOutput, rightMotorOutput)
    }

    fun swerveDrive(
        drivetrain: FalconSwerveDrivetrain<*>,
        forwardInput: Double,
        strafeInput: Double,
        rotationInput: Double,
        fieldRelative: Boolean
    ): ChassisSpeeds {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardInput, strafeInput, rotationInput, drivetrain.gyro() //drivetrain.robotPosition.rotation
        )
    }

    companion object {
        const val kQuickStopThreshold = 0.2
        const val kQuickStopAlpha = 0.1
        private const val kHighAdjustmentPower = 1.75 + 0.4375
        private const val kLowAdjustmentPower = 1.50
        private const val kMaxSpeed = 1.0
        private const val kHighPowerRotationScalar = 0.8
        private const val kLowPowerScalar = 0.5
        private const val kRotationExponent = 4.0
        private const val kPoleThreshold = 0.0
        private val kRobotRelativePoleThreshold = Math.toRadians(5.0)
        private const val kDeadband = 0.25
        private const val kRotationDeadband = 0.15
    }
}
