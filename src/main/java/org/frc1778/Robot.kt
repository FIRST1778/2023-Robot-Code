package org.frc1778

import com.github.ajalt.colormath.model.RGB
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.animation.BlinkAnimation
import org.frc1778.commands.lights.BalanceAnimation
import org.frc1778.lib.FalconTimedRobot
import org.frc1778.subsystems.*
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
    private val eventLoop = EventLoop()
    private val brakeModeLimitSwitchHit = BooleanEvent(
        eventLoop, Shooter.brakeModeSwitch::get
    )

    //    val alliance: DriverStation.Alliance = Alliance.Red


    val pdp = PowerDistribution(30, PowerDistribution.ModuleType.kRev)
    private val powerTab = Shuffleboard.getTab("Field")


    private var autonomousCommand: Command? = null

    val pcm = PneumaticHub(30)
    val compressor = pcm.makeCompressor()


    var driveInversion = when (alliance) {
        Alliance.Red -> -1
        else -> 1
    }

    init {
        +Vision
        +Drive
        +Shooter
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

        //TODO: These might be backwards
        brakeModeLimitSwitchHit.rising().ifHigh {
            Shooter.setBrakeMode(true)
        }
        brakeModeLimitSwitchHit.falling().ifHigh {
            Shooter.setBrakeMode(false)
        }

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
        eventLoop.poll()
    }

    override fun disabledInit() {
        Shooter.resetDesiredAngle()
    }

    override fun disabledPeriodic() {
//        Arm.resetDesiredExtension()
//        Arm.resetDesiredAngle()
//        Manipulator.resetDesiredAngle()
        Shooter.resetDesiredAngle()
    }

    override fun autonomousInit() {

        alliance = DriverStation.getAlliance()
        driveInversion = when (alliance) {
            Alliance.Red -> -1
            else -> 1
        }
        autonomousCommand = RobotContainer.getAutonomousCommand()
        autonomousCommand?.schedule()
        Shooter.resetDesiredAngle()


    }


    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
//        Shooter.setVoltage(3.0.volts)
        BalanceAnimation().schedule()
//        DotStar.setAnimation(
//                BlinkAnimation(
//                    RGB.from255(0,255,0), RGB, 3
//                )
//            )
//            DotStar.animateOn()
        Shooter.resetDesiredAngle()
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
