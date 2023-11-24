package org.frc1778.subsystems.drive

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.MountPoseConfigs
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.frc1778.Constants
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.subsystems.drive.swerve.AbstractSwerveDriveInputs
import org.ghrobotics.lib.subsystems.drive.swerve.SwerveDriveIO
import org.ghrobotics.lib.utils.Source

class SwerveDriveIOSim : SwerveDriveIO {

    private val pigeon2 = Pigeon2(Constants.DriveConstants.pigeonCanID).apply {
        configurator.apply(
            Pigeon2Configuration().withMountPose(
                MountPoseConfigs().withMountPoseYaw(-90.0)
            )
        )
    }

    private val moduleConstants = listOf(
        SwerveModuleConstants().withSwerveConstants(Constants.DriveConstants.topLeftSwerveModuleConstants)
            .withLocationX(Constants.DriveConstants.wheelBase / 2)
            .withLocationY(Constants.DriveConstants.trackWidth / 2),
        SwerveModuleConstants().withSwerveConstants(Constants.DriveConstants.topRightSwerveModuleConstants)
            .withLocationX(Constants.DriveConstants.wheelBase / 2)
            .withLocationY(-Constants.DriveConstants.trackWidth / 2),
        SwerveModuleConstants().withSwerveConstants(Constants.DriveConstants.bottomRightSwerveModuleConstants)
            .withLocationX(-Constants.DriveConstants.wheelBase / 2)
            .withLocationY(-Constants.DriveConstants.trackWidth / 2),
        SwerveModuleConstants().withSwerveConstants(Constants.DriveConstants.bottomLeftSwerveModuleConstants)
            .withLocationX(-Constants.DriveConstants.wheelBase / 2)
            .withLocationY(Constants.DriveConstants.trackWidth / 2),
    )

    private fun SwerveModuleConstants.withSwerveConstants(constants: org.ghrobotics.lib.subsystems.drive.swerve.SwerveModuleConstants): SwerveModuleConstants =
        this.withDriveMotorId(constants.kDriveTalonId)
            .withSteerMotorId(constants.kAzimuthTalonId)
            .withFeedbackSource(SwerveModuleConstants.SwerveModuleSteerFeedbackType.RemoteCANcoder)
            .withSpeedAt12VoltsMps(Constants.DriveConstants.maxSpeed.value)
            .withCANcoderOffset(constants.kAzimuthEncoderHomeOffset / 360)
            .withDriveMotorGearRatio(1/Constants.DriveConstants.driveReduction)
            .withSteerMotorGearRatio(Constants.DriveConstants.steerReduction)
            .withWheelRadius(constants.kWheelDiameter / 2)
            .withSteerMotorGains(
                Slot0Configs()
                    .withKP(constants.kAzimuthKp * 2)
            )


    private val modules = moduleConstants.map {
        SwerveModule(it, "rio", false).apply {
            driveMotor.configurator.apply(
                MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
            )
        }
    }

    private val sim = SimSwerveDrivetrain(
        listOf(
            Translation2d(Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2),
            Translation2d(Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2),
            Translation2d(-Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2),
            Translation2d(-Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2),
        ).toTypedArray(), pigeon2, SwerveDrivetrainConstants().withPigeon2Id(21), *moduleConstants.toTypedArray()
    )

    override val gyro: Source<Rotation2d> = { pigeon2.rotation2d }
    private var desiredStates = Array(4) { SwerveModuleState() }
    override val positions: Array<SwerveModulePosition>
        get() = Array(4) {
            modules[it].getPosition(true)
        }
    override val states: Array<SwerveModuleState>
        get() = Array(4) {
            modules[it].currentState
        }

    override val kinematics: SwerveDriveKinematics = sim.Kinem

    override fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        val states = kinematics.toSwerveModuleStates(chassisSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            Constants.DriveConstants.maxSpeed.value,
        )
        if (states.sumOf { it.speedMetersPerSecond } < 0.01 && (chassisSpeeds.vxMetersPerSecond != 0.0 || chassisSpeeds.vyMetersPerSecond != 0.0 || chassisSpeeds.omegaRadiansPerSecond != 0.0)) {
            println("Chassis Speeds but no Wheel Speeds")
        }
            setModuleStates(states)
        }

        private fun setModuleStates(states: Array<SwerveModuleState>) {
            desiredStates = states
            for (i in 0..3) {
                modules[i].apply(states[i], true)
            }
        }

        override fun setNeutral() {
            desiredStates = Array(4) { SwerveModuleState() }
            for (module in modules) {
                module.apply(SwerveModuleState(), true)
            }
        }

        override fun <T : AbstractSwerveDriveInputs> updateInputs(inputs: T) {
            sim.update(0.02, 12.0, *modules.toTypedArray())
            inputs.leftFrontDriveVoltage = modules[0].driveMotor.motorVoltage.value.volts
            inputs.rightFrontDriveVoltage = modules[1].driveMotor.motorVoltage.value.volts
            inputs.rightBackDriveVoltage = modules[2].driveMotor.motorVoltage.value.volts
            inputs.leftBackDriveVoltage = modules[3].driveMotor.motorVoltage.value.volts

            inputs.leftFrontDriveCurrent = modules[0].driveMotor.statorCurrent.value.amps
            inputs.rightFrontDriveCurrent = modules[1].driveMotor.statorCurrent.value.amps
            inputs.rightBackDriveCurrent = modules[2].driveMotor.statorCurrent.value.amps
            inputs.leftBackDriveCurrent = modules[3].driveMotor.statorCurrent.value.amps

            inputs.leftFrontSteerVoltage = modules[0].steerMotor.motorVoltage.value.volts
            inputs.rightFrontSteerVoltage = modules[1].steerMotor.motorVoltage.value.volts
            inputs.rightBackSteerVoltage = modules[2].steerMotor.motorVoltage.value.volts
            inputs.leftBackSteerVoltage = modules[3].steerMotor.motorVoltage.value.volts

            inputs.leftFrontSteerCurrent = modules[0].steerMotor.statorCurrent.value.amps
            inputs.rightFrontSteerCurrent = modules[1].steerMotor.statorCurrent.value.amps
            inputs.rightBackSteerCurrent = modules[2].steerMotor.statorCurrent.value.amps
            inputs.leftBackSteerCurrent = modules[3].steerMotor.statorCurrent.value.amps

            inputs.leftFrontPosition = modules[0].steerMotor.position.value.meters
            inputs.rightFrontPosition = modules[1].steerMotor.position.value.meters
            inputs.rightBackPosition = modules[2].steerMotor.position.value.meters
            inputs.leftBackPosition = modules[3].steerMotor.position.value.meters

            inputs.leftFrontRotation = modules[0].currentState.angle.radians.radians
            inputs.rightFrontRotation = modules[1].currentState.angle.radians.radians
            inputs.rightBackRotation = modules[2].currentState.angle.radians.radians
            inputs.leftBackRotation = modules[3].currentState.angle.radians.radians

            inputs.leftFrontVelocity = modules[0].driveMotor.velocity.value.meters / 1.seconds
            inputs.rightFrontVelocity = modules[1].driveMotor.velocity.value.meters / 1.seconds
            inputs.rightBackVelocity = modules[2].driveMotor.velocity.value.meters / 1.seconds
            inputs.leftBackVelocity = modules[3].driveMotor.velocity.value.meters / 1.seconds

            inputs.leftFrontFeedforward = 0.volts
            inputs.rightFrontFeedforward = 0.volts
            inputs.rightBackFeedforward = 0.volts
            inputs.leftBackFeedforward = 0.volts

            inputs.gyroRaw = pigeon2.angle.degrees

            inputs.chassisSpeeds = sim.Kinem.toChassisSpeeds(*states)

            inputs.states = states.toList()
            inputs.desiredStates = desiredStates.toList()


        }

    }