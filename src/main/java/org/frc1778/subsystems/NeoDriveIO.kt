package org.frc1778.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.frc1778.Constants
import org.frc1778.lib.swervedrive.FalconNeoSwerveModule
import org.frc1778.lib.swervedrive.SwerveDriveIO
import org.frc1778.lib.swervedrive.SwerveDriveInputs
import org.frc1778.subsystems.Drive.positions
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.utils.Source

class NeoDriveIO : SwerveDriveIO {
    val modules: List<FalconNeoSwerveModule> = listOf(
        FalconNeoSwerveModule(Constants.DriveConstants.topLeftSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.topRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomLeftSwerveModuleConstants),
    )

    val motorCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    init {
        for (module in modules.reversed()) {
            Constants.DriveConstants.driveTab.add(module.name, module).withSize(3, 4)
        }
        modules.forEach {
            it.setAngle(0.0)
        }
    }

    override fun updateInputs(inputs: SwerveDriveInputs) {
        inputs.leftFrontVoltage = 0.volts
        inputs.rightFrontVoltage = 0.volts
        inputs.rightBackVoltage = 0.volts
        inputs.leftBackVoltage = 0.volts

        inputs.leftFrontCurrent = 0.amps
        inputs.rightFrontCurrent = 0.amps
        inputs.rightBackCurrent = 0.amps
        inputs.leftBackCurrent = 0.amps

        inputs.leftFrontPosition = 0.meters
        inputs.rightFrontPosition = 0.meters
        inputs.rightBackPosition = 0.meters
        inputs.leftBackPosition = 0.meters

        inputs.leftFrontRotation = 0.radians
        inputs.rightFrontRotation = 0.radians
        inputs.rightBackRotation = 0.radians
        inputs.leftBackRotation = 0.radians

        inputs.leftFrontVelocity = 0.meters / 1.seconds
        inputs.rightFrontVelocity = 0.meters / 1.seconds
        inputs.rightBackVelocity = 0.meters / 1.seconds
        inputs.leftBackVelocity = 0.meters / 1.seconds

        inputs.leftFrontFeedforward = 0.volts
        inputs.rightFrontFeedforward = 0.volts
        inputs.rightBackFeedforward = 0.volts
        inputs.leftBackFeedforward = 0.volts

        inputs.gyroRaw = 0.0.radians
    }

    override fun setModuleStates(states: Array<SwerveModuleState>) {
        for (i in 0..3) {
            modules[i].setState(states[i])
        }
    }

    override fun setNeutral() {
        for (module in modules) {
            module.setNeutral()
        }
    }

    override val positions: Array<SwerveModulePosition>
        get() = modules.positions.toTypedArray()
    override val states: Array<SwerveModuleState>
        get() = Array(4) {
            SwerveModuleState(
                modules[it].driveVelocity.value, Rotation2d(modules[it].encoder.absolutePosition.value)
            )
        }

    // The gyro member is used for the SwerveDrivePoseEstimator and SwerveDriveOdometry.
    // This means we must use the raw yaw from the gyro instead of the odometry yaw (although
    // in general we prefer the latter over the former).
    override val gyro: Source<Rotation2d> = { Rotation2d(Gyro.yaw()) }
}