package org.frc1778.commands.intake

import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential

class IntakeToShooterCommand: FalconCommand(Intake) {

    override fun initialize() {
        if(!Intake.cubeStored()) {
            sequential {
                +IntakeSuckCommand()
                +this@IntakeToShooterCommand
            }.schedule()
        }
    }

    override fun execute() {
        Intake.retract()
        Intake.suck()
    }

    override fun end(interrupted: Boolean) {
        Intake.stop()
    }

    override fun isFinished(): Boolean {
        return !Intake.cubeStored()
    }

}