package org.frc1778.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.frc1778.Constants
import org.frc1778.commands.drive.TeleOpDriveCommand
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.lib.pathplanner.PathPoint
import org.frc1778.lib.swervedrive.FalconSwerveDrivetrain
import org.frc1778.lib.swervedrive.SwerveDriveIO
import org.frc1778.lib.swervedrive.SwerveDriveInputsAutoLogged
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.utils.Source
import org.littletonrobotics.junction.Logger
import kotlin.math.hypot

object Drive : FalconSwerveDrivetrain(), Sendable {
    var aprilTagsEnabled: Boolean = false

    var scoringPose: Pose2d? = null

    private const val maxVoltage = 12.0

    override val swerveDriveIO: SwerveDriveIO = NeoDriveIO()
    override val swerveDriveInputs: SwerveDriveInputsAutoLogged = SwerveDriveInputsAutoLogged()

    override fun lateInit() {
        super.lateInit()
        Constants.DriveConstants.driveTab.add("Drive", this).withSize(3, 4)
        defaultCommand = TeleOpDriveCommand()
    }

    override val wheelbase: Double = Constants.DriveConstants.wheelBase


    override val trackWidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed
    override val motorOutputLimiter: Source<Double> = {
        1.00
    }

    override val controller: HolonomicDriveController = HolonomicDriveController(
        PIDController(
            0.75, 0.0, 0.15

        ), PIDController(
            0.75, 0.0, 0.15
        ), ProfiledPIDController(
            0.2, 0.0, 0.02, TrapezoidProfile.Constraints(
                Constants.DriveConstants.maxAngularSpeed.value * 25.0,
                Constants.DriveConstants.maxAngularAcceleration.value * 18.5
            )
        )
    )

    fun getEstimatedCameraPose(previousEstimatedRobotPosition: Pose2d): Pair<Pose2d, Double>? {
        if (!aprilTagsEnabled) return null

        val result = Vision.getEstimatedGlobalPose(previousEstimatedRobotPosition)
        if (result == null || !result.isPresent) return null
        return result.get().estimatedPose.toPose2d() to result.get().timestampSeconds
    }


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

    val odometry: SwerveDriveOdometry = SwerveDriveOdometry(kinematics, swerveDriveIO.gyro(), swerveDriveIO.positions)

    // Estimator for the robot pose to integrate vision approximation,
    // Two vectors represent the std dv of the robot pose and the std dv of the camera pose.
    // Currently filled with default values.
    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        swerveDriveIO.gyro(),
        swerveDriveIO.positions,
        robotPosition,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9)
    )


    fun setPose(pose: Pose2d) {
        resetPosition(pose, swerveDriveIO.positions)
    }

    override fun resetPosition(pose: Pose2d, positions: Array<SwerveModulePosition>) {
        poseEstimator.resetPosition(swerveDriveIO.gyro(), positions, pose)
    }

    //TODO: Possibly use Pathfinding to generate a trajectory to our goal
    fun trajectoryToGoal(): PathPlannerTrajectory? {
        val currChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates().toTypedArray())
        if (scoringPose == null) return null
        return PathPlanner.generatePath(
            PathConstraints(maxSpeed.value, 3.0), PathPoint(
                robotPosition.translation,
                Transform2d(robotPosition, scoringPose).translation.angle,
                robotPosition.rotation,
                hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond)
            ), PathPoint(
                scoringPose!!.translation,
                Transform2d(scoringPose, robotPosition).translation.angle,
                robotPosition.rotation
            )
        )
    }

    fun trajectoryToPose(pose: Pose2d): Trajectory {
        val currChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates().toTypedArray())
        return PathPlanner.generatePath(
            PathConstraints(maxSpeed.value, 3.0), PathPoint(
                robotPosition.translation,
                Transform2d(robotPosition, scoringPose).translation.angle,
                robotPosition.rotation,
                hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond)
            ), PathPoint(
                pose.translation, Transform2d(pose, robotPosition).translation.angle, robotPosition.rotation
            )
        )
    }

    override fun periodic() {
        Logger.getInstance().processInputs("Drive IO", swerveDriveInputs)

        val cameraOut = getEstimatedCameraPose(robotPosition)
        if (cameraOut != null) {
            val (cameraEstimatedRobotPose, timeStamp) = cameraOut
            poseEstimator.addVisionMeasurement(cameraEstimatedRobotPose, timeStamp)
        }
        robotPosition = poseEstimator.update(swerveDriveIO.gyro(), swerveDriveIO.positions)

        field.robotPose = robotPosition

        FalconDashboard.robotHeading = robotPosition.rotation.radians
        FalconDashboard.robotX = robotPosition.translation.x_u.inFeet()
        FalconDashboard.robotY = robotPosition.translation.y_u.inFeet()
    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addDoubleProperty("Max Angular Speed", {
            Constants.DriveConstants.maxAngularSpeed.value * 17.0
        }, {})
        builder.addDoubleProperty("Max Angular Accel", {
            Constants.DriveConstants.maxAngularAcceleration.value * 10.0
        }, {})
    }

    fun swerveModuleStates(): List<SwerveModuleState> = swerveDriveIO.states.asList()
}

