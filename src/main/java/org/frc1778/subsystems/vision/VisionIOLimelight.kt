package org.frc1778.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.lib.LimelightHelpers
import org.frc1778.subsystems.drive.Drive
import org.ghrobotics.lib.utils.Source

class VisionIOLimelight : VisionIO {
    companion object {
        private const val CAMERA_NAME = "Eye of Sauron"
    }

    private var results: LimelightHelpers.LimelightResults? = null
    private var targetPoses: List<Pose3d>? = null
    private var robotPose3d: Pose3d? = null

    val targetDetected: Source<Boolean> = { LimelightHelpers.getTV(CAMERA_NAME) }
    private fun getLatestResults(): LimelightHelpers.LimelightResults? {
        return if (targetDetected()) results ?: kotlin.run {
            results = LimelightHelpers.getLatestResults(CAMERA_NAME)
            results!!
        } else null
    }

    override fun getTargetPoses3d(): List<Pose3d>? {
        return getLatestResults()?.let { result ->
            result.targetingResults.targets_Fiducials.map {
                it.targetPose_RobotSpace
            }
        }
    }

    override fun getRobotPose3d(): Pair<Pose3d, Double>? {
        return getLatestResults()?.let {
            (if (Vision.alliance == DriverStation.Alliance.Red) it.targetingResults.botPose3d_wpiRed
            else it.targetingResults.botPose3d_wpiRed) to it.targetingResults.timestamp_RIOFPGA_capture
        }
    }

    override fun updateInputs(inputs: VisionInputs) {
        inputs.targetDetected = targetDetected()
        inputs.targetPoses = getTargetPoses3d() ?: listOf(Pose3d())
        inputs.robotPose3d = getRobotPose3d()?.first ?: Pose3d(Drive.robotPosition)
        results = null
        if(!targetDetected()) {
            targetPoses = null
            robotPose3d = null
        }
    }


}