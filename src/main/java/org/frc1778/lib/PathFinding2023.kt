package org.frc1778.lib

import com.pathplanner.lib.PathPlannerTrajectory.Waypoint
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d

class PathFinding2023(val alliance: DriverStation.Alliance) {

    val nodes: Set<Node> = when (alliance) {
        DriverStation.Alliance.Red -> redNodes.toSet()
        else -> blueNodes.toSet()
    }


    inner class Node(
        val waypoint: PathPoint, val connections: Set<Node>, val open: Boolean = false, val area: Rectangle2d? = null
    ) {
        constructor(
            currentPose: Pose2d, currentSpeeds: ChassisSpeeds = ChassisSpeeds()
        ) : this(PathPoint.fromCurrentHolonomicState(currentPose, currentSpeeds),
            (if (nodes.any { it.area?.contains(currentPose.translation) == true }) nodes.filter {
                it.area?.contains(
                    currentPose.translation
                ) ?: false
            } else nodes.filter { it.open }).toSet(),
            false
        )

    }


    companion object {
        val blueNodes = listOf<Node>()

        val redNodes = listOf<Node>()

    }
}