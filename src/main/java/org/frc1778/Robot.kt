package org.frc1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.lights.TeleopLightCommand
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.lib.PathFinding2023
import org.frc1778.lib.pathplanner.PathConstraints
import org.frc1778.lib.pathplanner.PathPlanner
import org.frc1778.lib.pathplanner.PathPlannerTrajectory
import org.frc1778.lib.pathplanner.PathPoint
import org.frc1778.lib.pathplanner.server.PathPlannerServer
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.frc1778.subsystems.Vision
import org.frc1778.subsystems.Wrist
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import kotlin.system.measureTimeMillis

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
    private val eventLoop = EventLoop()
    private val brakeModeLimitSwitchHit = BooleanEvent(
        eventLoop, Wrist.brakeModeSwitch::get
    )

    //    val alliance: DriverStation.Alliance = Alliance.Red

    //    val alliance: DriverStation.Alliance = Alliance.Red


    val pdp = PowerDistribution(30, PowerDistribution.ModuleType.kRev)
    private val powerTab = Shuffleboard.getTab("Field")


    private var autonomousCommand: Command? = null

    private val pcm = PneumaticHub(30)
    private val compressor: Compressor = pcm.makeCompressor()


    var driveInversion = when (alliance) {
        Alliance.Red -> -1
        else -> 1
    }

    init {
        +Vision
        +Drive
        +Shooter
        +Lights
        +Intake
        +Wrist
        Wrist.setBrakeMode(true)

    }


    override fun robotInit() {
        SmartDashboard.setNetworkTableInstance(
            NetworkTableInstance.getDefault()
        )
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer

        //TODO: These might be backwards
        brakeModeLimitSwitchHit.rising().ifHigh {
            Wrist.setBrakeMode(true)
        }
        brakeModeLimitSwitchHit.falling().ifHigh {
            Wrist.setBrakeMode(false)
        }
        Wrist.setBrakeMode(true)
        PathPlannerServer.startServer(5811)


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
        Wrist.resetDesiredAngle()
        Lights.setAnimation(Lights.animations.random())
        Lights.animateOn()
    }

    override fun disabledPeriodic() {
//        Arm.resetDesiredExtension()
//        Arm.resetDesiredAngle()
//        Manipulator.resetDesiredAngle()
        Wrist.resetDesiredAngle()
        eventLoop.poll()
    }

    override fun autonomousInit() {
        TeleopLightCommand().schedule()
        Wrist.resetDesiredAngle()
        alliance = DriverStation.getAlliance()
        driveInversion = when (alliance) {
            Alliance.Red -> -1
            else -> 1
        }
        autonomousCommand = RobotContainer.getAutonomousCommand()
        autonomousCommand?.schedule()


    }


    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
//        Shooter.setVoltage(3.0.volts)
        autonomousCommand?.cancel()
        TeleopLightCommand().schedule()
        Wrist.resetDesiredAngle()
        alliance = DriverStation.getAlliance()

        val pathFinder: PathFinding2023 = PathFinding2023.fromJson(
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
            ), Alliance.Blue, alliance, setOf(5, 6)
        )!!


        var foundPath: List<PathPoint>?


        measureTimeMillis {
            foundPath = pathFinder.findPath(
                Pose2d(
                    Translation2d(
                        2.00, 2.5
                    ), Rotation2d.fromDegrees(180.0)
                ), ChassisSpeeds(0.0, 0.0, 0.0), Pose2d(
                    Translation2d(
                        12.5, 7.5
                    ), Rotation2d.fromDegrees(0.0)
                )

            )
        }.let { time ->
            println("Path Finding Time: $time ms")
        }


        lateinit var pathPlannerTrajectory: PathPlannerTrajectory

        measureTimeMillis {
            foundPath?.let {
                pathPlannerTrajectory = PathPlanner.generatePath(
                    PathConstraints(
                        Constants.DriveConstants.maxSpeed.value, Constants.DriveConstants.maxSpeed.value * 1.5
                    ),
                    it,
                )
            }
        }.let { time ->
            println("Trajectory Creating Time: $time ms")
        }

        measureTimeMillis {
            Drive.followTrajectory(pathPlannerTrajectory).schedule()
        }.let { time ->
            println("Trajectory Command Creation and Schedule Time: $time ms")
        }

//        PathPlannerServer.sendActivePath(pathPlannerTrajectory.states)

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
