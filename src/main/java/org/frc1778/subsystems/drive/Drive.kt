package org.frc1778.subsystems.drive

import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.frc1778.Constants
import org.frc1778.commands.drive.TeleOpDriveCommand
import org.frc1778.subsystems.vision.Vision
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.subsystems.drive.swerve.FalconSwerveDrivetrain
import org.ghrobotics.lib.subsystems.drive.swerve.SwerveDriveIO
import org.ghrobotics.lib.utils.Source
import org.littletonrobotics.junction.Logger


object Drive : FalconSwerveDrivetrain(), Sendable {
    var aprilTagsEnabled: Boolean = false

    var scoringPose: Pose2d? = null

    private const val maxVoltage = 12.0

    override var swerveDriveIO: SwerveDriveIO = if (isReal()) {
        SwerveDriveIOSparkMax()
    } else {
        SwerveDriveIOSim()
    }
    override val swerveDriveInputs: SwerveDriveInputsAutoLogged = SwerveDriveInputsAutoLogged()

    override fun lateInit() {
        super.lateInit()
        Constants.DriveConstants.driveTab.add("Drive", this).withSize(3, 4)
        defaultCommand = TeleOpDriveCommand()
    }

    override fun autoReset() {
//        resetPosition(Pose2d(), swerveDriveIO.positions)
    }

    override val wheelbase: Double = Constants.DriveConstants.wheelBase

    override val trackWidth: Double = Constants.DriveConstants.trackWidth
    override val maxSpeed: SIUnit<Velocity<Meter>> = Constants.DriveConstants.maxSpeed


    override val motorOutputLimiter: Source<Double> = {
        1.00
    }
    override val pathConstraints: PathConstraints = PathConstraints(
        Constants.DriveConstants.maxSpeed.value,
        maxSpeed.value * 3,
        Constants.DriveConstants.maxAngularSpeed.value,
        Constants.DriveConstants.maxAngularAcceleration.value * 30
    )

    val translationPIDConstants = if(isReal()) {
        PIDConstants(
            0.75, 0.0, 0.15

        )
    } else {
        // Sim doesn't have inertia so no k
        PIDConstants(
            2.5, 0.125, 0.25

        )
    }
    val rotationPIDConstants = PIDConstants(
        0.2, 0.0, 0.02
    )

    override val pathFollowingConfig: HolonomicPathFollowerConfig = HolonomicPathFollowerConfig(
        translationPIDConstants,
        rotationPIDConstants,
        maxSpeed.value,
        Constants.DriveConstants.driveBaseRadius,
        replanningConfig
    )

    override val controller = PPHolonomicDriveController(
        translationPIDConstants, rotationPIDConstants, maxSpeed.value, Constants.DriveConstants.driveBaseRadius
    )

    fun getEstimatedCameraPose(): Pair<Pose3d, Double>? {
        return Vision.getRobotPose3d()
    }


    override fun disableClosedLoopControl() {

    }

    override fun enableClosedLoopControl() {

    }

    val odometry: SwerveDriveOdometry =
        SwerveDriveOdometry(swerveDriveIO.kinematics, swerveDriveIO.gyro(), swerveDriveIO.positions)

    // Estimator for the robot pose to integrate vision approximation,
    // Two vectors represent the std dv of the robot pose and the std dv of the camera pose.
    // Currently filled with default values.
    private val poseEstimator = SwerveDrivePoseEstimator(
        swerveDriveIO.kinematics,
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
        controller.reset(pose, ChassisSpeeds())
        poseEstimator.resetPosition(swerveDriveIO.gyro(), positions, pose)
    }


    override fun periodic() {
        swerveDriveIO.updateInputs(swerveDriveInputs)
        Logger.processInputs("Drive IO", swerveDriveInputs)

        val cameraOut = getEstimatedCameraPose()
        if (cameraOut != null) {
            val (cameraEstimatedRobotPose, timeStamp) = cameraOut
            poseEstimator.addVisionMeasurement(cameraEstimatedRobotPose.toPose2d(), timeStamp)
        }

        val useSwervePositions = true
        if (isReal() || useSwervePositions) {
            robotPosition = poseEstimator.update(swerveDriveIO.gyro(), swerveDriveIO.positions)
        } else {
            robotPosition += Transform2d(
                swerveDriveInputs.desiredChassisSpeeds.vxMetersPerSecond * 0.02,
                swerveDriveInputs.desiredChassisSpeeds.vyMetersPerSecond * 0.02,
                Rotation2d.fromRadians(swerveDriveInputs.desiredChassisSpeeds.omegaRadiansPerSecond * 0.02)
            )
        }

        field.robotPose = robotPosition


        Logger.recordOutput(
            "Robot Position", doubleArrayOf(
                robotPosition.translation.x, robotPosition.translation.y, robotPosition.rotation.radians
            )
        )
        Logger.recordOutput(
            "Swerve Drive States", listOf(
                swerveDriveInputs.states[0].angle.radians,
                swerveDriveInputs.states[0].speedMetersPerSecond,
                swerveDriveInputs.states[1].angle.radians,
                swerveDriveInputs.states[1].speedMetersPerSecond,
                swerveDriveInputs.states[2].angle.radians,
                swerveDriveInputs.states[2].speedMetersPerSecond,
                swerveDriveInputs.states[3].angle.radians,
                swerveDriveInputs.states[3].speedMetersPerSecond
            ).toDoubleArray()
        )

        Logger.recordOutput(
            "Swerve Drive Desired States", listOf(
                swerveDriveInputs.desiredStates[0].angle.radians,
                swerveDriveInputs.desiredStates[0].speedMetersPerSecond,
                swerveDriveInputs.desiredStates[1].angle.radians,
                swerveDriveInputs.desiredStates[1].speedMetersPerSecond,
                swerveDriveInputs.desiredStates[2].angle.radians,
                swerveDriveInputs.desiredStates[2].speedMetersPerSecond,
                swerveDriveInputs.desiredStates[3].angle.radians,
                swerveDriveInputs.desiredStates[3].speedMetersPerSecond
            ).toDoubleArray()
        )

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

