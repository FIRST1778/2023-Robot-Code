/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.frc1778.lib

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.localization.TimePoseInterpolatableBuffer
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import java.util.*
import kotlin.collections.HashMap

abstract class FalconSwerveDrivetrain<T : AbstractFalconSwerveModule<*, *>> :
    TrajectoryTrackerSwerveDriveBase(), SensorlessCompatibleSubsystem {
    /**
     * The current inputs and outputs
     */
    protected val periodicIO = PeriodicIO()

    /**
     * Helper for different drive styles.
     */
    protected val driveHelper = FalconDriveHelper()

    protected abstract val poseEstimator: SwerveDrivePoseEstimator


    /**
     * Buffer for storing the pose over a span of time. This is useful for
     * Vision and latency compensation.
     */
    protected open val poseBuffer = TimePoseInterpolatableBuffer()


    /**
     * The left front motor
     */
    protected abstract val modules: List<T>

    abstract val wheelbase: Double

    abstract val trackWidth: Double

    abstract val maxSpeed: SIUnit<Velocity<Meter>>

    abstract val motorOutputLimiter: Source<Double>

    /**
     * The characterization for the swerve modules.
     */
    abstract val motorCharacterization: SimpleMotorFeedforward


    /**
     * The rotation source / gyro
     */
    abstract val gyro: Source<Rotation2d>

    /**
     * Get the robot's position on the field.
     */
    override var robotPosition: Pose2d = Pose2d()

    abstract fun getEstimatedCameraPose(previousEstimatedRobotPosition: Pose2d): Pair<Pose2d, Double>?

     fun swerveModuleStates(): List<SwerveModuleState> = List(4) {
            SwerveModuleState(
                modules[it].driveVelocity.value, Rotation2d(modules[it].encoder.absolutePosition.value)
            )
        }


    override fun periodic() {
//        periodicIO.leftFrontVoltage = modules[0].voltageOutput
//        periodicIO.rightFrontVoltage = modules[1].voltageOutput
//        periodicIO.rightBackVoltage = modules[2].voltageOutput
//        periodicIO.leftBackVoltage = modules[3].voltageOutput
//
//        periodicIO.leftFrontCurrent = modules[0].drawnCurrent
//        periodicIO.rightFrontCurrent = modules[1].drawnCurrent
//        periodicIO.rightBackCurrent = modules[2].drawnCurrent
//        periodicIO.leftBackCurrent = modules[3].drawnCurrent
//
//        periodicIO.leftFrontPosition = modules[0].drivePosition
//        periodicIO.rightFrontPosition = modules[1].drivePosition
//        periodicIO.rightBackPosition = modules[2].drivePosition
//        periodicIO.leftBackPosition = modules[3].drivePosition
//
//        periodicIO.leftFrontVelocity = modules[0].driveVelocity
//        periodicIO.rightFrontVelocity = modules[1].driveVelocity
//        periodicIO.rightBackVelocity = modules[2].driveVelocity
//        periodicIO.leftBackVelocity = modules[3].driveVelocity

        periodicIO.gyro = gyro()

        val feedForwards = Array(4) { 0.0.volts }
        feedForwards[0] = periodicIO.leftFrontFeedforward
        feedForwards[1] = periodicIO.rightFrontFeedforward
        feedForwards[2] = periodicIO.rightBackFeedforward
        feedForwards[3] = periodicIO.leftBackFeedforward

        periodicIO.positions = modules.positions.toTypedArray()


        val cameraOut = getEstimatedCameraPose(robotPosition)
        if (cameraOut != null) {
            val (cameraEstimatedRobotPose, timeStamp) = cameraOut
            poseEstimator.addVisionMeasurement(cameraEstimatedRobotPose, timeStamp)
        }

        robotPosition = poseEstimator.update(periodicIO.gyro, periodicIO.positions)


        poseBuffer[Timer.getFPGATimestamp().seconds] = robotPosition

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                modules.forEach { it.setNeutral() }
            }

            is Output.States -> {

                for (i in modules.indices) {
                    modules[i].setState(desiredOutput.states[i], feedForwards[i])
                }
            }
        }

        FalconDashboard.robotHeading = robotPosition.rotation.radians
        FalconDashboard.robotX = robotPosition.translation.x_u.inFeet()
        FalconDashboard.robotY = robotPosition.translation.y_u.inFeet()
    }

    /**
     * Represents periodic data
     */

    override fun lateInit() {
        resetPosition(Pose2d(), modules.positions.toTypedArray())
    }

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
        periodicIO.leftFrontFeedforward = 0.volts
        periodicIO.rightFrontFeedforward = 0.volts
        periodicIO.leftBackFeedforward = 0.volts
        periodicIO.rightBackFeedforward = 0.volts
    }

    override fun setOutputSI(
        states: Array<SwerveModuleState>
    ) {
        periodicIO.desiredOutput = Output.States(states)
    }


    @Deprecated("Pose Buffer is not consistent with the internally interpolated pose buffer in the poseEstimator")
    fun getPose(timestamp: SIUnit<Second> = Timer.getFPGATimestamp().seconds): Pose2d {
        return poseBuffer[timestamp] ?: kotlin.run {
            DriverStation.reportError("[FalconSD] Pose Buffer is Empty!", false)
            Pose2d()
        }
    }




    fun swerveDrive(forwardInput: Double, strafeInput: Double, rotationInput: Double, fieldRelative: Boolean) {
        val outputLimiter = motorOutputLimiter()
        val speeds = driveHelper.swerveDrive(
            this,
            forwardInput * outputLimiter,
            strafeInput * outputLimiter,
            rotationInput * outputLimiter,
            fieldRelative
        )
        val states = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed.value)
        periodicIO.desiredOutput = Output.States(states)

    }


    fun resetPosition(pose: Pose2d, positions: Array<SwerveModulePosition>) {
        poseEstimator.resetPosition(gyro(), positions, pose)
    }

    fun followTrajectory(trajectory: Trajectory, mirrored: Boolean = false) =
        SwerveTrajectoryTrackerCommand(this, Source(if (mirrored) trajectory.mirror() else trajectory))

    fun followTrajectory(trajectory: Trajectory, mirrored: BooleanSource) =
        SwerveTrajectoryTrackerCommand(this, mirrored.map(trajectory.mirror(), trajectory))

    fun followTrajectory(trajectory: Source<Trajectory>) = SwerveTrajectoryTrackerCommand(this, trajectory)

    fun followTrajectoryWithCommands(trajectory: Source<PathPlannerTrajectory>, eventMap: HashMap<String, Command>) =
        SwerveTrajectoryTrackerWithMarkersCommand(this, trajectory, eventMap)

    protected class PeriodicIO {
        var leftFrontVoltage: SIUnit<Volt> = 0.volts
        var rightFrontVoltage: SIUnit<Volt> = 0.volts
        var rightBackVoltage: SIUnit<Volt> = 0.volts
        var leftBackVoltage: SIUnit<Volt> = 0.volts

        var leftFrontCurrent: SIUnit<Ampere> = 0.amps
        var rightFrontCurrent: SIUnit<Ampere> = 0.amps
        var rightBackCurrent: SIUnit<Ampere> = 0.amps
        var leftBackCurrent: SIUnit<Ampere> = 0.amps

        var leftFrontPosition: SIUnit<Meter> = 0.meters
        var rightFrontPosition: SIUnit<Meter> = 0.meters
        var rightBackPosition: SIUnit<Meter> = 0.meters
        var leftBackPosition: SIUnit<Meter> = 0.meters

        var leftFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
        var rightFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
        var rightBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
        var leftBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds


        var gyro: Rotation2d = Rotation2d()

        var desiredOutput: Output = Output.Nothing

        var positions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }
        var leftFrontFeedforward: SIUnit<Volt> = 0.volts
        var rightFrontFeedforward: SIUnit<Volt> = 0.volts
        var rightBackFeedforward: SIUnit<Volt> = 0.volts
        var leftBackFeedforward: SIUnit<Volt> = 0.volts
    }

    /**
     * Represents the typical outputs for the drivetrain.
     */
    protected sealed class Output {
        // No outputs
        object Nothing : Output()

        class States(val states: Array<SwerveModuleState>) : Output()
    }

    val List<T>.positions: List<SwerveModulePosition>
        get() = List(4) {
            SwerveModulePosition(
                this[it].drivePosition.value, Rotation2d(this[it].encoder.absolutePosition.value)
            )
        }


}
