package org.frc1778.commands

import org.frc1778.subsystems.Intake
import org.frc1778.commands.intake.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.WaitCommand


class LoadShooter : FalconCommand(Intake) {
	lateinit var command: CommandBase


	override fun initialize() {
		command = sequential {
			+IntakeSuckCommand()
			if(Intake.cubeStored()) {
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