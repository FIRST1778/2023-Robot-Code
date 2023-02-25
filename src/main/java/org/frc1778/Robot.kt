package org.frc1778

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.ArmAngleCommand
import org.frc1778.commands.ExtensionCommand
import org.frc1778.commands.IntakeToggleCommand
import org.frc1778.commands.ZeroExtensionCommand
import org.frc1778.lib.DataLogger
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.lib.SwerveTrajectoryTrackerCommand
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Vision
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.wrappers.FalconSolenoid

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

    lateinit var angleCommand: ArmAngleCommand
    lateinit var zeroExtensionCommand: ZeroExtensionCommand
    lateinit var extensionCommand : ExtensionCommand

    private val trajectory: PathPlannerTrajectory = PathPlanner.loadPath("Trajectory Test", 4.00, 1.00)
    private lateinit var trajectoryCommand: SwerveTrajectoryTrackerCommand
    private var autonomousCommand: Command? = null

    val pcm = PneumaticHub(30)
    val compressor = pcm.makeCompressor()

    public var dataLogger = DataLogger("DataLogs")

    init {
        +Vision
        +Drive
        +Arm
    }


    override fun robotInit() {
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer
        SmartDashboard.setNetworkTableInstance(
                NetworkTableInstance.getDefault()
        )
        Drive.pigeon.yaw = 0.0
        field.getObject("traj").setTrajectory(trajectory)
        fieldTab.add("Field", field).withSize(8, 4)
    }


    override fun robotPeriodic() {
        field.robotPose = Drive.robotPosition

//        SmartDashboard.updateValues()


    }

    override fun disabledInit() {
//        compressor.disable()


    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {
        zeroExtensionCommand = ZeroExtensionCommand()
        zeroExtensionCommand.schedule()
        //angleCommand = ArmAngleCommand(0.0.degrees)
        //angleCommand.schedule()

        trajectoryCommand = Drive.followTrajectory(trajectory)
        Drive.setPose(trajectory.initialHolonomicPose)
        autonomousCommand = trajectoryCommand
        autonomousCommand?.schedule()
//        trajectoryCommand.schedule()

    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
        //angleCommand = ArmAngleCommand(90.0.degrees)
        //angleCommand.schedule()

        //extensionCommand = ExtensionCommand(0.5.meters)
        //extensionCommand.schedule()
        var command = sequential{
            +ZeroExtensionCommand()
            +ExtensionCommand(0.0.meters)
            +ArmAngleCommand(50.0.degrees) // hopper angle
            +ExtensionCommand(0.3.meters) // hopper extension
        }
        command.schedule()

        Drive.setPose(trajectory.initialHolonomicPose)
//        compressor.enableAnalog(
//            30.0,
//            40.0
//        )

    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
        dataLogger.log()
    }

    override fun simulationPeriodic() {

    }
    override fun simulationInit() {

    }
}
