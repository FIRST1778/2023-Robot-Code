package org.frc1778.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.frc1778.Constants
import org.frc1778.commands.TeleOpDriveCommand
import org.frc1778.lib.FalconSwerveDrivetrain
import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.subsystems.AbstractFalconSwerveModule
import org.ghrobotics.lib.utils.Source

object Drive : FalconSwerveDrivetrain<FalconNeoSwerveModule>() {

    val pigeon = Pigeon2(Constants.DriveConstants.pigeonCanID).apply {

    }


    override val modules: List<FalconNeoSwerveModule> = listOf(
        FalconNeoSwerveModule(Constants.DriveConstants.topLeftSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.topRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomLeftSwerveModuleConstants),

    )
    override val wheelbase: Double = Constants.DriveConstants.wheelBase

    override val trackwidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed

    override val leftFrontCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val rightFrontCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val leftBackCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val rightBackCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val gyro: Source<Rotation2d> = { Rotation2d(pigeon.yaw) }

    override val controller: RamseteController = RamseteController(7.0, 2.0)


    /**
     * Wheel Positions in order
     * - Front Left
     * - Front Right
     * - Back Right
     * - Back Left
     */
    override val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        Translation2d(wheelbase / 2, trackwidth / 2),
        Translation2d(-wheelbase / 2, trackwidth / 2),
        Translation2d(-wheelbase / 2, -trackwidth / 2),
        Translation2d(wheelbase / 2, -trackwidth / 2),
    )

    override val odometry: SwerveDriveOdometry = SwerveDriveOdometry(kinematics, gyro(), Array(4) {
        SwerveModulePosition(modules[it].drivePosition.value, Rotation2d(modules[it].anglePosition.value))
    })

    init {
        for(i in modules.indices) {
//            SmartDashboard.putNumber("Encoder $i", modules[i].encoder.rawAbsolutePosition.value)
        }
//        defaultCommand = TeleOpDriveCommand()
    }

    override fun disableClosedLoopControl() {
        TODO("Not yet implemented")
    }

    override fun enableClosedLoopControl() {
        TODO("Not yet implemented")
    }
}