package org.frc1778.subsystems

import edu.wpi.first.apriltag.AprilTagDetector
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.frc1778.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

object Vision : FalconSubsystem() {

    private val camera = PhotonCamera(Constants.VisionConstants.cameraName)
    private val photonPoseEstimator = PhotonPoseEstimator(
        Constants.VisionConstants.APIRL_TAG_FIELD_LAYOUT,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        camera,
        Constants.VisionConstants.CAMERA_ROBOT_TRANSFORM,

    )

    fun getEstimatedGlobalPose(previousEstimatedRobotPosition: Pose2d): Optional<EstimatedRobotPose>? {
        photonPoseEstimator.setReferencePose(previousEstimatedRobotPosition)
        return photonPoseEstimator.update()
    }



}