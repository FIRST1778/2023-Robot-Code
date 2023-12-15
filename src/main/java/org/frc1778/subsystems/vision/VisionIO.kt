package org.frc1778.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.ghrobotics.lib.junction.AutoLog


@AutoLog
open class VisionInputs {
    var targetDetected: Boolean = false
    var targetPoses: List<Pose3d> = listOf()
    var robotPose3d: Pose3d = Pose3d()
}

interface VisionIO {
    fun getTargetPoses3d(): List<Pose3d>?
    fun getRobotPose3d(): Pair<Pose3d, Double>?
    fun updateInputs(inputs: VisionInputs)
}
