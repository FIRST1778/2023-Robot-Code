package org.frc1778.lib

import edu.wpi.first.math.MathUtil
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
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.properties.Delegates

class PathFinding2023(
    nodes: Set<Node>, private val openIndexes: Set<Int>, private val zones: Set<Rectangle2d>
) {

    private val _nodes: Set<Node> = nodes

    constructor(
        waypoints: List<PathPlannerTrajectory.Waypoint>, openIndexes: Set<Int>, zones: Set<Rectangle2d>
    ) : this(waypoints.map { waypoint ->
        Node(PathPoint(
            waypoint.anchorPoint, waypoint.heading, waypoint.holonomicRotation
        ).apply {
            if (waypoint.nextControl != null) {
                withNextControlLength(waypoint.nextControlLength)
            }
            if (waypoint.prevControl != null) {
                withPrevControlLength(waypoint.prevControlLength)
            }
        })
    }.toSet(), openIndexes, zones)

    init {
        _nodes.forEachIndexed(this::setConnections)
        _nodes.filter { it.open }.forEach {
            it.connections += _nodes.filter { it.open }.toSet()
        }
    }


    private fun setConnections(index: Int, node: Node) {
        node.open = index in openIndexes
        val nodeZones: Set<Rectangle2d> = zones.filter { it.contains(node.pathPoint.position) }.toSet()
        node.connections =
            _nodes.filter { node -> nodeZones.any { zone -> zone.contains(node.pathPoint.position) } }.toSet()
    }

    private fun setConnections(node: Node) {

        val nodeZones: Set<Rectangle2d> = zones.filter { it.contains(node.pathPoint.position) }.toSet()
        if (nodeZones.isNotEmpty()) {
            node.open = false
            node.connections =
                _nodes.filter { node -> nodeZones.any { zone -> zone.contains(node.pathPoint.position) } }.toSet()
        } else {
            node.connections = _nodes.filter { it.open }.toSet()
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
        end.pathPoint.withPrevControlLength(0.01)
        setConnections(start)
        setConnections(end)

        val endZone = try {
            zones.first { it.contains(end.pathPoint.position) }
        } catch (e: NoSuchElementException) {
            return null
        }
        _nodes.filter { endZone.contains(it.pathPoint.position) }.forEach {
            it.connections += end
        }
        val openList = (_nodes.toMutableList() + start + end) as MutableList<Node>



        start.f = 0.0
        start.g = 0.0
        start.h = 0.0
        start.parent = start

        while (openList.isNotEmpty()) {
            openList.sortBy { it.f }
            val currNode = openList.first()
            openList.remove(currNode)
            currNode.closed = true

            for (nextNode in currNode.connections.filter { !it.closed }) {
                if (nextNode == end) {
                    end.parent = currNode
                    return end.tracePath(start)
                } else {
                    val gNew = nextNode.g + (nextNode distanceTo currNode)
                    val hNew =
                        DISTANCE_WEIGHT * (nextNode distanceTo end) + HEADING_WEIGHT * (currNode diffHeading nextNode)
                    val hNewInverted =
                        DISTANCE_WEIGHT * (nextNode distanceTo end) + HEADING_WEIGHT * (nextNode.invertedNode() diffHeading currNode)
                    val fNew = gNew + min(hNew, hNewInverted)
                    if (nextNode.f == Double.MAX_VALUE || nextNode.f > fNew) {
                        if (hNewInverted < hNew) nextNode.invertHeading()
                        nextNode.f = fNew
                        nextNode.g = gNew
                        nextNode.h = hNew
                        nextNode.parent = currNode
                        openList.add(nextNode)
                    }
                }

            }

        }
        return null
    }


    companion object {
        private const val FIELD_LENGTH_METERS: Double = 16.54
        const val DISTANCE_WEIGHT: Double = .125
        const val HEADING_WEIGHT = .75

        private val PathPlannerTrajectory.Waypoint.heading: Rotation2d
            get() {
                return Rotation2d(
                    this.anchorPoint.x - (this.nextControl?.x ?: this.anchorPoint.x),
                    this.anchorPoint.y - (this.nextControl?.y ?: this.anchorPoint.y)
                )
            }

        private val PathPlannerTrajectory.Waypoint.prevControlLength: Double
            get() {
                return prevControl?.let {
                    sqrt(
                        (this.anchorPoint.x - it.x).pow(2) + (this.anchorPoint.y - it.y).pow(2)
                    )
                } ?: 0.01
            }


        private val PathPlannerTrajectory.Waypoint.nextControlLength: Double
            get() {
                return nextControl?.let {
                    sqrt(
                        (this.anchorPoint.x - it.x).pow(2) + (this.anchorPoint.y - it.y).pow(2)
                    )
                } ?: 0.01
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
    var pathPoint: PathPoint
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
        println(this.h)
        if (this.parent == start) return mutableListOf(this.parent!!, this)
        return (this.parent!!.tracePath(start)) + this
    }

    infix fun distanceTo(other: Node): Double {
        val dx = this.pathPoint.position.x - other.pathPoint.position.x
        val dy = this.pathPoint.position.y - other.pathPoint.position.y
        return hypot(dx.pow(2), dy.pow(2))
    }

    infix fun diffHeading(other: Node): Double {
        return abs(
            MathUtil.inputModulus(
                this.pathPoint.heading.radians - other.pathPoint.heading.radians, -ANGLE_WRAP, ANGLE_WRAP
            )
        )
    }

    fun invertHeading() {
        this.pathPoint = PathPoint(
            this.pathPoint.position,
            this.pathPoint.heading.plus(Rotation2d.fromDegrees(180.0)),
            this.pathPoint.holonomicRotation
        ).apply {
            if (pathPoint.nextControlLength > 0) {
                withPrevControlLength(pathPoint.nextControlLength)
            }
            if (pathPoint.prevControlLength > 0) {
                withNextControlLength(pathPoint.prevControlLength)
            }
        }
    }

    fun invertedNode(): Node {
        return Node(pathPoint).apply { invertHeading() }
    }


    companion object {
        private const val ANGLE_WRAP = (PI - -PI) / 2.0
    }

}
