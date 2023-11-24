package org.frc1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.intake.IntakeToShooterCommand
import org.frc1778.subsystems.intake.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential


class ShooterLoadCommand : FalconCommand(Intake) {
	lateinit var command: Command

	override fun initialize() {
		command = sequential {
			if(!Intake.cubeStored()) {
				+IntakeSuckCommand()
				+WaitCommand(.25)
			}
			+IntakeToShooterCommand()
		}
		command.initialize()
	}

	override fun execute() {
		command.execute()
	}

	override fun cancel() {
		super.cancel()
		command.cancel()
	}

	override fun end(interrupted: Boolean) {
		command.end(interrupted)
	}

	override fun isFinished(): Boolean {
		return command.isFinished()
	}

	
}