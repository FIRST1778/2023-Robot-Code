/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.frc1778.lib.swervedrive

import com.gamingnight.junction.AutoLog
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.lib.FalconDriveHelper
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.lib.swervedrive.commands.SwerveTrajectoryGroupTrackerCommand
import org.frc1778.lib.swervedrive.commands.SwerveTrajectoryTrackerCommand
import org.frc1778.lib.swervedrive.commands.SwerveTrajectoryTrackerWithMarkersCommand
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.ghrobotics.lib.utils.Source

abstract class FalconSwerveDrivetrain : TrajectoryTrackerSwerveDriveBase(),
    SensorlessCompatibleSubsystem {
    /**
     * The current inputs and outputs
     */
    abstract val swerveDriveIO: SwerveDriveIO
    abstract val swerveDriveInputs: SwerveDriveInputs

    /**
     * Helper for different drive styles.
     */
    protected val driveHelper = FalconDriveHelper()

    abstract val wheelbase: Double

    abstract val trackWidth: Double

    abstract val maxSpeed: SIUnit<Velocity<Meter>>

    abstract val motorOutputLimiter: Source<Double>

    /**
     * Get the robot's position on the field. Up to the user to update the position.
     */
    override var robotPosition: Pose2d = Pose2d()

    val field = Field2d()
    private val fieldTab = Shuffleboard.getTab("Field")

    abstract fun resetPosition(pose: Pose2d, positions: Array<SwerveModulePosition>)

    fun resetPosition(newPose: Pose2d) {
        resetPosition(newPose, swerveDriveIO.positions)
    }

    fun setTrajectory(traj: Trajectory) {
        field.getObject("traj").setTrajectory(traj)
    }


    /**
     * Represents periodic data
     */
    override fun lateInit() {
        resetPosition(Pose2d(), swerveDriveIO.positions)
        fieldTab.add("Field", field).withSize(8, 4)
    }

    override fun setNeutral() {
        swerveDriveIO.setNeutral()
    }

    override fun setOutputSI(
        states: Array<SwerveModuleState>
    ) {
       swerveDriveIO.setModuleStates(states)
    }



    fun swerveDrive(forwardInput: Double, strafeInput: Double, rotationInput: Double, fieldRelative: Boolean = true) {
        val outputLimiter = motorOutputLimiter()
        val speeds = driveHelper.swerveDrive(
            this,
            forwardInput * outputLimiter,
            strafeInput * outputLimiter,
            rotationInput * outputLimiter * .75,
            fieldRelative
        )
        val states = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed.value)
        swerveDriveIO.setModuleStates(states)
    }


    fun followTrajectory(trajectory: PathPlannerTrajectory, mirrored: Boolean = false) = SwerveTrajectoryTrackerCommand(
        this, Source((if (mirrored) trajectory.mirror() else trajectory) as PathPlannerTrajectory)
    )

    fun followTrajectory(trajectory: Source<PathPlannerTrajectory>) = SwerveTrajectoryTrackerCommand(this, trajectory)

    fun followTrajectoryWithCommands(trajectory: PathPlannerTrajectory, eventMap: HashMap<String, Command>) =
        SwerveTrajectoryTrackerWithMarkersCommand(this, trajectory, eventMap)

    fun followTrajectoryGroupWithCommands(
        trajectories: List<PathPlannerTrajectory>, eventMap: HashMap<String, Command>
    ) = SwerveTrajectoryGroupTrackerCommand(this, trajectories, eventMap)

    val List<AbstractFalconSwerveModule<*, *>>.positions: List<SwerveModulePosition>
        get() = List(4) {
            SwerveModulePosition(
                this[it].drivePosition.value, Rotation2d(this[it].encoder.absolutePosition.value)
            )
        }

}

interface SwerveDriveIO {

    fun updateInputs(inputs: SwerveDriveInputs)
    fun setModuleStates(states: Array<SwerveModuleState>)
    fun setNeutral()

    val positions: Array<SwerveModulePosition>

    val states: Array<SwerveModuleState>

    val gyro: Source<Rotation2d>
}

@AutoLog
open class SwerveDriveInputs {
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

    var leftFrontRotation: SIUnit<Radian> = 0.radians
    var rightFrontRotation: SIUnit<Radian> = 0.radians
    var rightBackRotation: SIUnit<Radian> = 0.radians
    var leftBackRotation: SIUnit<Radian> = 0.radians

    var leftFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    var rightFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    var rightBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    var leftBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds

    var leftFrontFeedforward: SIUnit<Volt> = 0.volts
    var rightFrontFeedforward: SIUnit<Volt> = 0.volts
    var rightBackFeedforward: SIUnit<Volt> = 0.volts
    var leftBackFeedforward: SIUnit<Volt> = 0.volts

    var gyroRaw: SIUnit<Radian> = 0.0.radians
}
