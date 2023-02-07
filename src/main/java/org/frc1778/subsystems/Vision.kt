package org.frc1778.subsystems

import edu.wpi.first.math.geometry.Pose2d
import org.frc1778.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import java.util.*

object Vision : FalconSubsystem() {

    private val camera = PhotonCamera(Constants.VisionConstants.cameraName)
    private val photonPoseEstimator = PhotonPoseEstimator(
        Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        camera,
        Constants.VisionConstants.CAMERA_ROBOT_TRANSFORM,

    )

    fun getEstimatedGlobalPose(previousEstimatedRobotPosition: Pose2d): Optional<EstimatedRobotPose>? {
        photonPoseEstimator.setReferencePose(previousEstimatedRobotPosition)
        return photonPoseEstimator.update()
    }



}