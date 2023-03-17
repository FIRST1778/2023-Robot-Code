package org.frc1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.PlaceGameObjectCommand
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.lib.PathFinding2023
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.server.PathPlannerServer
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.DotStar
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.frc1778.subsystems.Vision
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import kotlin.properties.Delegates

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
    var alliance: Alliance = DriverStation.getAlliance()

    //    val alliance: DriverStation.Alliance = Alliance.Red


    val robotHeadingOnStart = 0.0

    val pdp = PowerDistribution(30, PowerDistribution.ModuleType.kRev)
    private val powerTab = Shuffleboard.getTab("Field")


    private var autonomousCommand: Command? = null


    val pcm = PneumaticHub(30)
    val compressor = pcm.makeCompressor()


    val driveInversion = when (alliance) {
        Alliance.Red -> -1
        else -> 1
    }


    var scoringLevel by Delegates.observable(Level.Top) { _, oldValue, newValue ->
        if (oldValue != newValue) {
            placeGameObjectCommand.level = newValue
        }
    }

    //TODO: We will probably need to perform more actions on this state Change
    var gamePiece: GamePiece by Delegates.observable(GamePiece.Cone) { _, oldValue, newValue ->
        if (oldValue != newValue) {
            placeGameObjectCommand = PlaceGameObjectCommand(scoringLevel, newValue, scoringStation, scoringSide)
        }
    }

    var scoringStation: Station by Delegates.observable(
        when (DriverStation.getLocation()) {
            1 -> Station.Left
            2 -> Station.Center
            3 -> Station.Right
            else -> Station.Center
        }
    ) { _, oldValue, newValue ->
        if (oldValue != newValue) {
            placeGameObjectCommand = PlaceGameObjectCommand(scoringLevel, gamePiece, newValue, scoringSide)
        }
    }

    var scoringSide: Side by Delegates.observable(Side.Right) { _, oldValue, newValue ->
        if (oldValue != newValue) {
            placeGameObjectCommand = PlaceGameObjectCommand(scoringLevel, gamePiece, scoringStation, newValue)
        }
    }

    var placeGameObjectCommand: PlaceGameObjectCommand =
        PlaceGameObjectCommand(scoringLevel, gamePiece, scoringStation, scoringSide)


    init {
        +Vision
        +Drive
        +Arm
        +Manipulator
        +DotStar
        +Intake
    }


    override fun robotInit() {
        SmartDashboard.setNetworkTableInstance(
            NetworkTableInstance.getDefault()
        )
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer

        PathPlannerServer.startServer(5811)





        Drive.setPose(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(robotHeadingOnStart)))
//        Drive.pigeon.yaw = 0.0
//        field.getObject("traj").setTrajectory(trajectory)

        compressor.enableAnalog(95.0, 115.0)
        powerTab.add("PDP", pdp).withSize(2, 2)

//        Arm.initialize()
//        Manipulator.initialize()
    }


    override fun robotPeriodic() {

        Shuffleboard.update()
        Controls.driverController.update()
        Controls.operatorControllerRed.update()
        Controls.operatorControllerBlue.update()
    }

    override fun disabledInit() {
//        compressor.disable()
    }

    override fun disabledPeriodic() {
//        Arm.resetDesiredExtension()
//        Arm.resetDesiredAngle()
//        Manipulator.resetDesiredAngle()
    }

    override fun autonomousInit() {

        alliance = DriverStation.getAlliance()
        autonomousCommand = RobotContainer.getAutonomousCommand()
        autonomousCommand?.schedule()


    }


    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        alliance = DriverStation.getAlliance()


        val pathFinder = PathFinding2023.fromJson(
            "Nodes Blue", setOf(0, 4, 5, 6), setOf(
                Rectangle2d(
                    Translation2d(1.5, 5.45), Translation2d(5.75, 4.00)
                ), Rectangle2d(
                    Translation2d(5.75, 1.45), Translation2d(1.5, 0.0)
                ), Rectangle2d(
                    Translation2d(1.5, 5.45), Translation2d(2.75, 0.0)
                ), Rectangle2d(
                    Translation2d(9.85, 8.00), Translation2d(16.2, 5.5)
                )
            ), Alliance.Blue
        )!!

        val foundPath = pathFinder.findPath(
            Pose2d(
                Translation2d(
                    8.00, 4.75
                ), Rotation2d.fromDegrees(0.0)
            ), ChassisSpeeds(), Pose2d(
                Translation2d(
                    2.00, 2.5
                ), Rotation2d.fromDegrees(180.0)
            )

        )


        val pathPlannerTrajectory = PathPlanner.generatePath(
            PathConstraints(2.0, 1.5),
            foundPath,
        )

        Drive.setTrajectory(
            pathPlannerTrajectory
        )
        Drive.followTrajectory(pathPlannerTrajectory).schedule()

        PathPlannerServer.sendActivePath(pathPlannerTrajectory.states)
//        Arm.resetDesiredAngle()
//        Arm.resetDesiredExtension()
//        Manipulator.resetDesiredAngle()
//        Arm.resetIsZeroed()
//        Drive.resetPosition(
//            when (alliance) {
//                Alliance.Blue -> {
//                    when (DriverStation.getLocation()) {
//                        1 -> Pose2d(
//                            Translation2d(1.85, 4.40), Rotation2d.fromDegrees(180.0)
//                        )
//
//                        2 -> Pose2d(
//                            Translation2d(1.85, 2.7), Rotation2d.fromDegrees(180.0)
//                        )
//
//                        else -> Pose2d(
//                            Translation2d(1.85, 1.05), Rotation2d.fromDegrees(180.0)
//                        )
//
//
//                    }
//                }
//
//                else -> {
//                    when (DriverStation.getLocation()) {
//                        1 -> Pose2d(
//                        Translation2d(14.75, 4.40), Rotation2d.fromDegrees(0.0)
//                    )
//
//                        2 -> Pose2d(
//                        Translation2d(14.75, 2.7), Rotation2d.fromDegrees(0.0)
//                    )
//
//                        else -> Pose2d(
//                        Translation2d(14.75, 1.05), Rotation2d.fromDegrees(0.0)
//                    )
//                    }
//                }
//            }, Drive.modules.positions.toTypedArray()
//        )
//        ArmAngleCommand(110.0.degrees).schedule()
//        ZeroExtensionCommand().schedule()
        // DO NOT REMOVE

    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
//        Manipulator.angleMotor.setVoltage(12.0.volts)
    }

    override fun simulationPeriodic() {

    }

    override fun simulationInit() {

    }
}
