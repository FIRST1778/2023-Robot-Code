package org.frc1778.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import com.pathplanner.lib.auto.RamseteAutoBuilder
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.frc1778.Constants
import org.frc1778.commands.TeleOpDriveCommand
import org.frc1778.lib.FalconSwerveDrivetrain
import org.frc1778.lib.SwerveModuleConstants
import org.frc1778.subsystems.Drive.modules
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.subsystems.AbstractFalconSwerveModule
import org.ghrobotics.lib.utils.Source

object Drive : FalconSwerveDrivetrain<FalconNeoSwerveModule>() {
    private val pigeon = Pigeon2(Constants.DriveConstants.pigeonCanID)

    private val maxVoltage = 12.0

    private var motorOutputLimiterEntry: GenericEntry =
        Constants.DriveConstants.driveTab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                mapOf(
                    "min" to 0.0, "max" to 100.0, "Block increment" to 10.0
                )
            ).entry!!

    public override val modules: List<FalconNeoSwerveModule> = listOf(
        FalconNeoSwerveModule(Constants.DriveConstants.topLeftSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.topRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomRightSwerveModuleConstants),
        FalconNeoSwerveModule(Constants.DriveConstants.bottomLeftSwerveModuleConstants),

        )

    init {

        for (module in modules.reversed()) {
            Constants.DriveConstants.driveTab.add(module.name, module).withSize(3, 4)
        }
        pigeon.configMountPose(Pigeon2.AxisDirection.PositiveX, Pigeon2.AxisDirection.PositiveZ, 500)
        pigeon.yaw = 0.0
        modules.forEach {
            it.setAngle(0.0)
        }
        defaultCommand = TeleOpDriveCommand()
    }

    override val wheelbase: Double = Constants.DriveConstants.wheelBase


    override val trackwidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed
    override val motorOutputLimiter: Source<Double> = {
        motorOutputLimiterEntry.getDouble(100.0) / 100.0
    }

    override val leftFrontCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val rightFrontCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val leftBackCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val rightBackCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val gyro: Source<Rotation2d> = { Rotation2d.fromDegrees(pigeon.yaw) }

    override val controller: RamseteController = RamseteController(
        .5, .125

    )



    /**
     * Wheel Positions in order
     * - Front Left
     * - Front Right
     * - Back Right
     * - Back Left
     */
    override val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        Translation2d(wheelbase / 2, trackwidth / 2),
        Translation2d(wheelbase / 2, -trackwidth / 2),
        Translation2d(-wheelbase / 2, -trackwidth / 2),
        Translation2d(-wheelbase / 2, trackwidth / 2),
    )

    override val odometry: SwerveDriveOdometry = SwerveDriveOdometry(kinematics, gyro(), Array(4) {
        SwerveModulePosition(modules[it].drivePosition.value, Rotation2d(modules[it].anglePosition.value))
    })

    fun setPose(pose: Pose2d) {
        odometry.resetPosition(gyro(), modules.positions.toTypedArray(), pose)
    }





    override fun disableClosedLoopControl() {
        TODO("Not yet implemented")
    }

    override fun enableClosedLoopControl() {
        TODO("Not yet implemented")
    }


}

