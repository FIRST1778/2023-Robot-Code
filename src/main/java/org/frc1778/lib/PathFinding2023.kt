package org.frc1778.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import org.frc1778.Robot
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.lib.pathplanner.PathPoint
import org.frc1778.lib.pathplanner.simple.JSONObject
import org.frc1778.lib.pathplanner.simple.parser.JSONParser
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.properties.Delegates

class PathFinding2023(
    private val nodes: Set<Node>, private val openIndexes: Set<Int>, private val zones: Set<Rectangle2d>
) {

    constructor(
        waypoints: List<PathPlannerTrajectory.Waypoint>, openIndexes: Set<Int>, zones: Set<Rectangle2d>
    ) : this(waypoints.map { waypoint ->
        Node(
            PathPoint(
                waypoint.anchorPoint, waypoint.heading, waypoint.holonomicRotation
            )
        )
    }.toSet(), openIndexes, zones)

    init {
        nodes.forEachIndexed(this::setConnections)
    }


    private fun setConnections(index: Int, node: Node) {
        node.open = index in openIndexes
        val nodeZones: Set<Rectangle2d> = zones.filter { it.contains(node.pathPoint.position) }.toSet()
        node.connections =
            nodes.filter { node -> nodeZones.any { zone -> zone.contains(node.pathPoint.position) } }.toSet()
    }

    private fun setConnections(node: Node) {

        val nodeZones: Set<Rectangle2d> = zones.filter { it.contains(node.pathPoint.position) }.toSet()
        if (nodeZones.isNotEmpty()) {
            node.open = false
            node.connections =
                nodes.filter { node -> nodeZones.any { zone -> zone.contains(node.pathPoint.position) } }.toSet()
        } else {
            node.connections = nodes.filter { it.open }.toSet()
            node.open = true
        }
    }

    fun findPath(start: Pose2d, currentSpeeds: ChassisSpeeds, end: Pose2d): List<PathPoint>? {
        return optimize(
            Node(start, currentSpeeds), Node(end)
        )?.map {
            it.pathPoint
        }
    }


    private fun optimize(start: Node, end: Node): List<Node>? {
        setConnections(start)
        setConnections(end)

        val endZone = zones.first { it.contains(end.pathPoint.position) }
        nodes.filter { endZone.contains(it.pathPoint.position) }.forEach {
            it.connections += end
        }
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
                    val path = end.tracePath(start)
                    return path
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


    companion object {
        private const val FIELD_LENGTH_METERS: Double = 16.54
        const val DISTANCE_WEIGHT = 1.0
        const val HEADING_WEIGHT = 0.5

        private val PathPlannerTrajectory.Waypoint.heading: Rotation2d
            get() {
                return Rotation2d(
                    this.anchorPoint.x - (this.nextControl?.x ?: this.anchorPoint.x),
                    this.anchorPoint.y - (this.nextControl?.y ?: this.anchorPoint.y)
                )
            }

        fun fromJson(
            name: String, openIndexes: Set<Int>, zones: Set<Rectangle2d>, from: Alliance, to: Alliance = Robot.alliance
        ): PathFinding2023? {
            try {
                BufferedReader(
                    FileReader(
                        File(Filesystem.getDeployDirectory(), "pathplanner/$name.path")
                    )
                ).use { br ->
                    val fileContentBuilder = StringBuilder()
                    var line: String?
                    while (br.readLine().also { line = it } != null) {
                        fileContentBuilder.append(line)
                    }
                    val fileContent = fileContentBuilder.toString()
                    val json = JSONParser().parse(fileContent) as JSONObject
                    val waypoints = PathPlannerTrajectory.transformWaypointsForAlliance(
                        PathPlanner.getWaypointsFromJson(json), to, from
                    )




                    return PathFinding2023(waypoints, openIndexes, transformZonesForAlliance(zones, from, to))
                }
            } catch (e: Exception) {
                e.printStackTrace()
                return null
            }
        }

        private fun transformZonesForAlliance(
            zones: Set<Rectangle2d>, from: Alliance, to: Alliance
        ): Set<Rectangle2d> {
            if (from != to) {
                val transformDirection: Int = if (to == Alliance.Red) {
                    1
                } else {
                    -1
                }
                return zones.map {
                    val topLeft = it.topLeft
                    val bottomRight = it.bottomRight

                    val transformedTopLeft = Translation2d(
                        FIELD_LENGTH_METERS / 2 + transformDirection * (FIELD_LENGTH_METERS / 2 - topLeft.x), topLeft.y
                    )
                    val transformedBottomRight = Translation2d(
                        FIELD_LENGTH_METERS / 2 + transformDirection * (FIELD_LENGTH_METERS / 2 - bottomRight.x),
                        bottomRight.y
                    )
                    Rectangle2d(transformedTopLeft, transformedBottomRight)
                }.toSet()
            }
            return zones
        }
    }


}

class Node(
    val pathPoint: PathPoint
) {

    var f: Double = Double.MAX_VALUE
    var g: Double = 0.0
    var h: Double = 0.0
    var parent: Node? = null
    var closed: Boolean = false
    lateinit var connections: Set<Node>
    var open by Delegates.notNull<Boolean>()

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
        PathPoint.fromCurrentHolonomicState(currentPose, currentSpeeds)
    )


    fun traceLengthTo(start: Node): Double {
        return this.tracePath(start).windowed(2).sumOf {
            it.first() distanceTo it.last()
        }
    }

    fun tracePath(start: Node): List<Node> {
        if (this.parent == start) return mutableListOf( this.parent!!, this)
        return  (this.parent!!.tracePath(start)) + this
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