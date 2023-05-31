package org.frc1778.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import org.frc1778.Constants
import org.frc1778.commands.drive.TeleOpDriveCommand
import org.frc1778.lib.DataLogger
import org.frc1778.lib.FalconNeoSwerveModule
import org.frc1778.lib.FalconSwerveDrivetrain
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.lib.pathplanner.PathPoint
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.utils.Source
import kotlin.math.hypot

object Drive : FalconSwerveDrivetrain<FalconNeoSwerveModule>(), Sendable{
    var aprilTagsEnabled: Boolean = false

    var scoringPose: Pose2d? = null

    private const val maxVoltage = 12.0

    private var motorOutputLimiterEntry: GenericEntry =
        Constants.DriveConstants.driveTab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                mapOf(
                    "min" to 0.0, "max" to 100.0, "Block increment" to 10.0
                )
            ).entry!!

    val driveLogger = DataLogger("Drive")
    init {
        driveLogger.add("X") { robotPosition.translation.x }
        driveLogger.add("Y") { robotPosition.translation.y }
        driveLogger.add("Rotation") {robotPosition.rotation.degrees}
    }

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
        Constants.DriveConstants.driveTab.add("Drive", this).withSize(3,4)
        modules.forEach {
            it.setAngle(0.0)
        }
        defaultCommand = TeleOpDriveCommand()
    }

    override val wheelbase: Double = Constants.DriveConstants.wheelBase


    override val trackWidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed
    override val motorOutputLimiter: Source<Double> = {
//        motorOutputLimiterEntry.getDouble(100.0) / 100.0
        1.00
    }

    override val motorCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    // The gyro member is used for the SwerveDrivePoseEstimator and SwerveDriveOdometry.
    // This means we must use the raw yaw from the gyro instead of the odometry yaw (although
    // in general we prefer the latter over the former).
    override val gyro: Source<Rotation2d> = { Rotation2d(Gyro.yaw()) }

    override fun getEstimatedCameraPose(previousEstimatedRobotPosition: Pose2d): Pair<Pose2d, Double>? {
        if (!aprilTagsEnabled)
            return null

        val result = Vision.getEstimatedGlobalPose(previousEstimatedRobotPosition)
        if (result == null || !result.isPresent)
            return null
        return result.get().estimatedPose.toPose2d() to result.get().timestampSeconds
    }

    //TODO: Tune Holonomic Drive Controller
    override val controller: HolonomicDriveController = HolonomicDriveController(
        PIDController(
            0.75,
            0.0,
            0.15

        ),
        PIDController(
            0.75,
            0.0,
            0.15
        ),
        ProfiledPIDController(
            0.2,
            0.0,
            0.02,
            TrapezoidProfile.Constraints(
                Constants.DriveConstants.maxAngularSpeed.value * 25.0,
                Constants.DriveConstants.maxAngularAcceleration.value * 18.5
            )
        )
    )



    /**
     * Wheel Positions in order
     * - Front Left
     * - Front Right
     * - Back Right
     * - Back Left
     */
    override val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        Translation2d(wheelbase / 2, trackWidth / 2),
        Translation2d(wheelbase / 2, -trackWidth / 2),
        Translation2d(-wheelbase / 2, -trackWidth / 2),
        Translation2d(-wheelbase / 2, trackWidth / 2),
    )

    override fun disableClosedLoopControl() {

    }

    override fun enableClosedLoopControl() {

    }

    val odometry: SwerveDriveOdometry = SwerveDriveOdometry(kinematics, gyro(), Array(4) {
        SwerveModulePosition(modules[it].drivePosition.value, Rotation2d(modules[it].anglePosition.value))
    })


    // Estimator for the robot pose to integrate vision approximation,
    // Two vectors represent the std dv of the robot pose and the std dv of the camera pose.
    // Currently filled with default values.
    override val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyro(),
        modules.positions.toTypedArray(),
        robotPosition,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9)
    )


    fun setPose(pose: Pose2d) {
        resetPosition(pose, modules.positions.toTypedArray())
    }


    //TODO: Possibly use Pathfinding to generate a trajectory to our goal
    fun trajectoryToGoal(): PathPlannerTrajectory? {
        val currChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates().toTypedArray())
        if(scoringPose == null) return null
        return PathPlanner.generatePath(
            PathConstraints(maxSpeed.value, 3.0),
            PathPoint(
                robotPosition.translation,
                Transform2d(robotPosition, scoringPose).translation.angle,
                robotPosition.rotation,
                hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond)
            ),
            PathPoint(
                scoringPose!!.translation,
                Transform2d(scoringPose, robotPosition).translation.angle,
                robotPosition.rotation
            )
        )
    }

    fun trajectoryToPose(pose: Pose2d) : Trajectory {
        val currChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates().toTypedArray())
        return PathPlanner.generatePath(
            PathConstraints(maxSpeed.value, 3.0),
            PathPoint(
                robotPosition.translation,
                Transform2d(robotPosition, scoringPose).translation.angle,
                robotPosition.rotation,
                hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond)
            ),
            PathPoint(
                pose.translation,
                Transform2d(pose, robotPosition).translation.angle,
                robotPosition.rotation
            )
        )
    }

//    override fun periodic() {
//        super.periodic() //DONT REMOVE
//        driveLogger.log()
//    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addDoubleProperty("Max Angular Speed", {Constants.DriveConstants.maxAngularSpeed.value * 17.0
        }, {})
        builder.addDoubleProperty("Max Angular Accel", {Constants.DriveConstants.maxAngularAcceleration.value * 10.0
        }, {})
    }
}

