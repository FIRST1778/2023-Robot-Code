package org.frc1778

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.frc1778.lib.SwerveTrajectoryTrackerCommand
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Vision
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryGenerator
import org.frc1778.lib.PathFinder
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : FalconTimedRobot() {


    private val field = Field2d()
    private val fieldTab = Shuffleboard.getTab("Field")

    //        private val trajectory: PathPlannerTrajectory = PathPlanner.loadPath("Trajectory Test", 4.00, 1.00)
    private lateinit var trajectoryCommand: SwerveTrajectoryTrackerCommand
    private var autonomousCommand: Command? = null

    //    val pcm = PneumaticHub(30)
//    val compressor = pcm.makeCompressor()
//
//    val sol = FalconDoubleSolenoid(
//        1,
//        0,
//        PneumaticsModuleType.REVPH,
//        30
//    )
    init {
        +Drive
        +Lights
        +Vision
    }


    override fun robotInit() {
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer
        SmartDashboard.setNetworkTableInstance(
            NetworkTableInstance.getDefault()
        )
        Drive.pigeon.yaw = 0.0
        //<editor-fold desc=" Path Gen Stuff">
        val pf = PathFinder(
            (36.25).inches,
//            .25.meters,
            //Red Substation
            Rectangle2d(
                Translation2d(0.00, 8.00), Translation2d(3.32, 5.50)
            ),
            Rectangle2d(
                Translation2d(3.32, 8.00), Translation2d(6.70, 6.77)
            ),
            //Red Scoring Zone
            Rectangle2d(
                Translation2d(11.67, 0.00), Translation2d(15.15, 5.50)
            ),
            //Blue Charge Station
            Rectangle2d(
                Translation2d(4.86, 1.52), Translation2d(2.87, 3.96)
            ),
        )
        val trajectoryConfig = FalconTrajectoryConfig(
            SIUnit(7.00), SIUnit(7.00)
        ).addConstraints(
            SwerveDriveKinematicsConstraint(
                Drive.kinematics, Constants.DriveConstants.maxSpeed.value
            )
        )
        val points = pf.findPath(
            Pose2d(
                Translation2d(2.25, 2.75), Rotation2d(
                    Math.toRadians(0.0)
                )
            ),
            Pose2d(
                Translation2d(15.00, 6.75), Rotation2d(
                    Math.toRadians(0.0)
                )
//                Translation2d(9.00, 4.50), Rotation2d(0.0)
            ),
        )
        val trajectory = points?.let {
            infix fun Pose2d.headingTo(other: Pose2d): Rotation2d {
                val relative = other.relativeTo(this)
                return Rotation2d(relative.translation.x, relative.translation.y)
            }

            infix fun Pose2d.distanceTo(other: Pose2d): Double =
                sqrt((this.x - other.x).pow(2) + (this.y - other.y).pow(2))


            val pathPoints: MutableList<PathPoint> = mutableListOf()
            pathPoints.add(
                points.take(2).let {
                        (firstPose, nextPose) ->
                    PathPoint(
                        firstPose.translation,
                        firstPose headingTo nextPose,
                        firstPose.rotation
                    ).withPrevControlLength(
                        sqrt(firstPose distanceTo nextPose)
                    )
                }
            )
            points.windowed(3).mapTo(pathPoints) { (prevPose, currPose, nextPose) ->
                PathPoint(
                    currPose.translation, prevPose headingTo currPose, currPose.rotation
                ).withControlLengths(
                    sqrt(prevPose distanceTo currPose),
                    sqrt(currPose distanceTo  nextPose)
                )
            }
            pathPoints.add(
                points.takeLast(2).let {
                    (prevPose, lastPose) ->
                    PathPoint(
                        lastPose.translation,
                        prevPose headingTo lastPose,
                        lastPose.rotation
                    ).withPrevControlLength(
                        sqrt(prevPose distanceTo lastPose)
                    )
                }
            )


            PathPlanner.generatePath(
                PathConstraints(7.00, 7.00), pathPoints
            )
        } ?: PathPlannerTrajectory()
//        val trajectory = points?.let {
//            FalconTrajectoryGenerator.generateTrajectory(
//                it, trajectoryConfig
//            )
//        } ?: Trajectory()
        //</editor-fold>
        field.getObject("traj").setTrajectory(trajectory)
        fieldTab.add("Field", field).withSize(8, 4)
        Drive.robotPosition = trajectory.initialPose
    }


    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        field.robotPose = Drive.robotPosition

//        SmartDashboard.updateValues()

    }

    override fun disabledInit() {
//        compressor.disable()

    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {
//        trajectoryCommand = Drive.followTrajectory(trajectory)
//        Drive.setPose(trajectory.initialHolonomicPose)
        autonomousCommand = trajectoryCommand
        autonomousCommand?.schedule()
//        trajectoryCommand.schedule()

    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
//        Drive.setPose(trajectory.initialHolonomicPose)
//        compressor.enableAnalog(
//            30.0,
//            40.0
//        )

    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
//        println(compressor.pressure)
//        Controls.operatorController.update()

    }

    override fun simulationInit() {

    }

    override fun simulationPeriodic() {

    }

}
