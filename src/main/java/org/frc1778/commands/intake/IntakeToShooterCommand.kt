package org.frc1778.commands.intake

import org.frc1778.Level
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
        if(Shooter.getScoringLevel() == Level.None) {
            Intake.suck()
            Shooter.suck()
        }
    }

    override fun end(interrupted: Boolean) {
        Intake.stop()
        Shooter.stopWheels()
    }

    override fun isFinished(): Boolean {
        return Shooter.cubeStored
    }
}