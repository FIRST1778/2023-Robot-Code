package org.frc1778

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.*
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.commands.lights.TeleopLightCommand
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.frc1778.subsystems.Vision
import org.frc1778.subsystems.Wrist

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

        brakeModeLimitSwitchHit.rising().ifHigh {
            Wrist.setBrakeMode(true)
        }
        brakeModeLimitSwitchHit.falling().ifHigh {
            Wrist.setBrakeMode(false)
        }
        Wrist.setBrakeMode(true)

//        field.getObject("traj").setTrajectory(trajectory)

        compressor.enableAnalog(95.0, 115.0)
        powerTab.add("PDP", pdp).withSize(2, 2)

//        Arm.initialize()
//        Manipulator.initialize()
    }


    override fun robotPeriodic() {

        Shuffleboard.update()
        Controls.xboxController.update()
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
    }

    var rumbleEnabled = true
    var matchTime = 0.0
    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {

    }

    override fun simulationPeriodic() {

    }

    override fun simulationInit() {

    }
}
