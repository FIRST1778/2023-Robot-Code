package org.frc1778.lib

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow

class PathFinding2023(private val alliance: Alliance) {

    val nodes: MutableSet<Node> = when (alliance) {
        Alliance.Red -> redNodes.toMutableSet()
        else -> blueNodes.toMutableSet()
    }

    fun findPath(start: Pose2d, currentSpeeds: ChassisSpeeds, end: Pose2d): List<PathPoint>? {
        return optimize(
            Node(start, currentSpeeds),
            Node(end)
        )?.map {
            it.pathPoint
        }
    }


    private fun optimize(start: Node, end: Node): List<Node>? {
        val openList = (nodes.toMutableList() + start + end) as MutableList<Node>

        start.f = 0.0
        start.g = 0.0
        start.h = 0.0
        start.parent = start

        while (openList.isNotEmpty()) {
            openList.sortBy { it.f }
            val currNode = openList.first()
            openList.remove(currNode)
            currNode.closed = true

            for (node in currNode.connections.filter { !it.closed }) {
                if (node == end) {
                    end.parent = currNode
                    return end.tracePath(start)
                } else {
                    val gNew = node.g + (node distanceTo currNode)
                    val hNew = DISTANCE_WEIGHT * (node distanceTo end) + HEADING_WEIGHT * (currNode diffHeading node)
                    val fNew = gNew + hNew
                    if (node.f == Double.MAX_VALUE || node.f > fNew) {
                        node.f = fNew
                        node.g = gNew
                        node.h = hNew
                        node.parent = currNode
                        openList.add(node)
                    }
                }

            }

        }
        return null
    }


    inner class Node(
        val pathPoint: PathPoint, private val open: Boolean = false, private val area: Rectangle2d? = null
    ) {

        var f: Double = Double.MAX_VALUE
        var g: Double = 0.0
        var h: Double = 0.0
        var parent: Node? = null
        var closed: Boolean = false

        fun reset() {
            f = Double.MAX_VALUE
            g = 0.0
            h = 0.0
            parent = null
            closed = false
        }

        constructor(
            currentPose: Pose2d, currentSpeeds: ChassisSpeeds = ChassisSpeeds()
        ) : this(
            PathPoint.fromCurrentHolonomicState(currentPose, currentSpeeds), false
        )

        val connections: Set<Node> = (if (nodes.any { it.area?.contains(pathPoint.position) == true }) nodes.filter {
            it.area?.contains(
                pathPoint.position
            ) ?: false
        } else nodes.filter { it.open }).toSet()

        fun traceLengthTo(start: Node): Double {
            return this.tracePath(start).windowed(2).sumOf {
                it.first() distanceTo it.last()
            }
        }

        fun tracePath(start: Node): List<Node> {
            if (this.parent == start) return mutableListOf(this, this.parent!!)
            return mutableListOf(this) + (this.parent!!.tracePath(start))
        }

        infix fun distanceTo(other: Node): Double {
            val dx = this.pathPoint.position.x - other.pathPoint.position.x
            val dy = this.pathPoint.position.y - other.pathPoint.position.y
            return hypot(dx.pow(2), dy.pow(2))
        }

        infix fun diffHeading(other: Node): Double {
            return abs(this.pathPoint.heading.degrees - other.pathPoint.heading.degrees)
        }


    }


    companion object {
        val blueNodes = listOf<Node>()
        val redNodes = listOf<Node>()

        private const val DISTANCE_WEIGHT = 1.0
        private const val HEADING_WEIGHT = 1.0

    }
}