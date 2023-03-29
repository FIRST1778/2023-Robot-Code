package org.frc1778.commands.intake

import org.frc1778.Level
import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand


class IntakeSpitCommand : FalconCommand(Intake) {
    override fun initialize() {
        Intake.spit()
        Intake.retract()
    }
    override fun execute(){
        if(Shooter.getScoringLevel() == Level.None){
            Shooter.spit()
        }
    }

    override fun cancel() {
        Intake.stop()
        if(Shooter.currentCommand != ShooterShootCommand()){
            Shooter.stopWheels()
        }
        super.cancel()
    }
}