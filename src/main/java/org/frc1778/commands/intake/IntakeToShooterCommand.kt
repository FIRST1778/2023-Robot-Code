package org.frc1778.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import org.frc1778.Level
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential

class IntakeToShooterCommand : FalconCommand(Intake) {

    override fun initialize() {
        Intake.retract()
        Intake.suck()
    }

    override fun execute() {
        Intake.retract()
        if(Shooter.getScoringLevel() == Level.None) {
            Intake.suck()
        }
    }


    override fun end(interrupted: Boolean) {
        Intake.stop()
    }

    override fun cancel() {
        super.cancel()
        end(true)
    }

    override fun isFinished(): Boolean {
        return Shooter.cubeStored
    }
}