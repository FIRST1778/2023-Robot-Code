package org.frc1778.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import org.frc1778.Constants
import org.frc1778.commands.PlaceGameObjectDriveCommand
import org.frc1778.commands.TeleOpDriveCommand
import org.frc1778.commands.SwerveTrajectoryTrackerCommand
import org.frc1778.lib.FalconNeoSwerveModule
import org.frc1778.lib.FalconSwerveDrivetrain
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.utils.Source
import kotlin.math.hypot

object Drive : FalconSwerveDrivetrain<FalconNeoSwerveModule>() {
    var scoringPose: Pose2d? = null

    val pigeon = Pigeon2(Constants.DriveConstants.pigeonCanID)

    private const val maxVoltage = 12.0

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
        modules.forEach {
            it.setAngle(0.0)
        }
        //defaultCommand = TeleOpDriveCommand()
    }

    override val wheelbase: Double = Constants.DriveConstants.wheelBase


    override val trackWidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed
    override val motorOutputLimiter: Source<Double> = {
        motorOutputLimiterEntry.getDouble(100.0) / 100.0
    }

    override val motorCharacterization: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    override val gyro: Source<Rotation2d> = { Rotation2d.fromDegrees(pigeon.yaw) }

    var gamePiecePlacementTrajectoryFollowCommand: PlaceGameObjectDriveCommand? = null

    override fun getEstimatedCameraPose(previousEstimatedRobotPosition: Pose2d): Pair<Pose2d, Double>? {
        val result = Vision.getEstimatedGlobalPose(previousEstimatedRobotPosition)
        if (result != null) {
            if(result.isPresent) {
                return result.get().estimatedPose.toPose2d() to result.get().timestampSeconds
            }
        }
        return null
    }

    //TODO: Tune Holonomic Drive Controller
    override val controller: HolonomicDriveController = HolonomicDriveController(
        PIDController(
            0.0,
            0.0,
            0.0
        ),
        PIDController(
            0.0,
            0.0,
            0.0
        ),
        ProfiledPIDController(
            0.0,
            0.0,
            0.0,
            TrapezoidProfile.Constraints(
                Constants.DriveConstants.maxSpeed.value,
                Constants.DriveConstants.maxAngularSpeed.value
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
    fun trajectoryToGoal(): Trajectory? {
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



}

