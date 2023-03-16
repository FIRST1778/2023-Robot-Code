package org.frc1778.commands.manipulator

import edu.wpi.first.wpilibj2.command.Command
import org.frc1778.GamePiece
import org.frc1778.Robot
import org.frc1778.commands.arm.ArmAngleCommand
import org.frc1778.commands.arm.ArmExtensionCommand
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters

class ManipulatorToHopperCommand() : FalconCommand(Arm, Manipulator, Intake) {

    private var desiredExtension: SIUnit<Meter> = 0.0.meters
    private val desiredAngle: SIUnit<Radian> = 0.0.radians //TODO
    private lateinit var command: Command


    override fun initialize() {
        //TODO
        desiredExtension = when (Robot.gamePiece) {
            GamePiece.Cone -> 0.0.meters
            GamePiece.Cube -> 0.0.meters
        }

        val command = sequential {
            +ArmExtensionCommand(0.0.meters)
            +ArmAngleCommand(desiredAngle)
            +ArmExtensionCommand(desiredExtension)
        }.schedule()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }


}