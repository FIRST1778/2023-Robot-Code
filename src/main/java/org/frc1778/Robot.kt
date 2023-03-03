package org.frc1778

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.ArmAngleCommand
import org.frc1778.commands.ExtensionCommand
import org.frc1778.commands.IntakeToggleCommand
import org.frc1778.commands.PlaceGameObjectCommand
import org.frc1778.commands.SwerveTrajectoryTrackerCommand
import org.frc1778.commands.ZeroExtensionCommand
import org.frc1778.lib.DataLogger
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.subsystems.*
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.wrappers.FalconSolenoid
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
    val alliance: DriverStation.Alliance = DriverStation.getAlliance()
    private val field = Field2d()
    private val fieldTab = Shuffleboard.getTab("Field")

    lateinit var zeroExtensionCommand: ZeroExtensionCommand

    private val trajectory: PathPlannerTrajectory = PathPlanner.loadPath("Trajectory Test", 4.00, 1.00)
    private lateinit var trajectoryCommand: SwerveTrajectoryTrackerCommand
    private var autonomousCommand: Command? = null

    val pcm = PneumaticHub(30)
    val compressor = pcm.makeCompressor()

    var dataLogger = DataLogger("DataLogs")


    var scoringLevel by Delegates.observable(Level.Top) { _, oldValue, newValue ->
        if (oldValue != newValue) {
            placeGameObjectCommand.level = newValue
        }
    }

    //TODO: We will probably need to perform more actions on this state Change
    var gamePiece: GamePiece by Delegates.observable(GamePiece.Cone) { _, oldValue, newValue ->
        if(oldValue != newValue) {
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
        if(oldValue != newValue) {
            placeGameObjectCommand = PlaceGameObjectCommand(scoringLevel, gamePiece, newValue, scoringSide)
        }
    }

    var scoringSide: Side by Delegates.observable(Side.Right) { _, oldValue, newValue ->
        if(oldValue != newValue) {
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
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer
        SmartDashboard.setNetworkTableInstance(
            NetworkTableInstance.getDefault()
        )
        Drive.pigeon.yaw = 0.0
        field.getObject("traj").setTrajectory(trajectory)
        fieldTab.add("Field", field).withSize(8, 4)

        Arm.extensionControlEnabled = false
        Arm.angleControlEnabled = false
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
        //zeroExtensionCommand = ZeroExtensionCommand()
        //zeroExtensionCommand.schedule()

        //trajectoryCommand = Drive.followTrajectory(trajectory)
        //Drive.setPose(trajectory.initialHolonomicPose)
        //autonomousCommand = trajectoryCommand
        //autonomousCommand?.schedule()
//        trajectoryCommand.schedule()

    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        //autonomousCommand?.cancel()

        //Drive.setPose(trajectory.initialHolonomicPose)
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
