package org.frc1778

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.GenericHID
import org.frc1778.commands.shooter.ShooterLoadCommand
import org.frc1778.commands.drive.BalanceCommand
import org.frc1778.commands.intake.IntakeLineBreakOverrideCommand
import org.frc1778.commands.intake.IntakeLowerCommand
import org.frc1778.commands.intake.IntakeSpitCommand
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.commands.shooter.ShooterSuckCommand
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.wrappers.hid.mapControls
import org.ghrobotics.lib.wrappers.hid.FalconHIDBuilder
import kotlin.math.abs
import kotlin.math.withSign

object Controls {
    private val driverHID = Joystick(0)
    private val xboxHID = Joystick(1)
    private val redHID  = Joystick(2)
    private val blueHID = Joystick(3)
    private val steamDeckHID = Joystick(4)

    private val driverController = FalconHIDBuilder(driverHID).build()

    private val xboxController = addXboxControls(xboxHID).build()
    private val steamDeckController = addXboxControls(steamDeckHID).build()

    // The Steam deck and the XBox controller use the same mappings with
    // the same button IDs.
    private fun addXboxControls(j: Joystick) = FalconHIDBuilder(j).apply {
        button(1) { changeOn(ShooterAngleCommand(Level.Bottom)) }  // A
        button(2) { changeOn(ShooterAngleCommand(Level.None)) }    // B
        button(3) { changeOn(ShooterAngleCommand(Level.Middle)) }  // X
        button(4) { changeOn(ShooterAngleCommand(Level.Top)) }     // Y
        button(7) { changeOn(IntakeLineBreakOverrideCommand()) }   // 2 squares
        button(8) { changeOn(ShooterAngleCommand(Level.THREE_POINT)) } // hamburger
        axisButton(2, 0.7) {  // LT
            change(ShooterShootCommand())
            changeOff { ShooterAngleCommand(Level.None).schedule() }
        }
        axisButton(3, 0.7) {
            change(
                sequential {
                    +ShooterAngleCommand(Level.None)
                    +parallelDeadline(ShooterSuckCommand()) {
                        +ShooterLoadCommand()
                    }
                }
            )
        }
    }

    private val redController = redHID.mapControls {
        button(1) { change(IntakeSpitCommand()) }
        button(4) { change(IntakeSuckCommand()) }
        button(5) { change(IntakeLowerCommand()) }

        button(2) {
            change(ShooterShootCommand())
            changeOff {
                ShooterAngleCommand(Level.None).schedule()
            }
        }

        button(3) {
            change(
                sequential {
                    +ShooterAngleCommand(Level.None)
                    +parallelDeadline(ShooterSuckCommand()) {
                        +ShooterLoadCommand()
                    }
                }
            )
        }
    }

    private val blueController = blueHID.mapControls {
        button(1) { changeOn(IntakeLineBreakOverrideCommand()) }
        button(4) { changeOn(ShooterAngleCommand(Level.Bottom)) }
        button(5) { changeOn(ShooterAngleCommand(Level.Middle)) }
        button(6) { changeOn(ShooterAngleCommand(Level.Top)) }
        button(7) { changeOn(ShooterAngleCommand(Level.None)) }
        button(8) { changeOn(ShooterAngleCommand(Level.THREE_POINT)) }
        button(10) { change(BalanceCommand()) }
        button(2) {}
        button(3) {}
        button(11) {}
        axisButton(1, -1.0) {}
    }

    fun update() {
        driverController.update()
        xboxController.update()
        redController.update()
        blueController.update()
        steamDeckController.update()
    }

    fun rumble(n: Double) {
        driverHID.setRumble(GenericHID.RumbleType.kBothRumble, n)
    }

    data class DriveState(
        val dx: Double, val dy: Double,
        val rot: Double, val align: Boolean
    )

    fun driveState(): DriveState {
        if (driverHID.isConnected()) {
            return DriveState(
                dx = driverController.getRawAxis(2)(),
                dy = driverController.getRawAxis(3)(),
                rot = driverController.getRawAxis(0)(),
                align = driverController.getRawButton(1)()
            )
        } else {
            return DriveState(
                dx = steamDeckController.getRawAxis(1)(),
                dy = steamDeckController.getRawAxis(0)(),
                rot = steamDeckController.getRawAxis(4)(),
                align = steamDeckController.getRawButton(10)()
            )
        }
    }
}
