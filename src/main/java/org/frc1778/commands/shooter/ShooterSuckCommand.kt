package org.frc1778.commands.shooter

import org.frc1778.Level
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterSuckCommand : FalconCommand(Shooter, Intake){
    override fun execute() {
        if(!Shooter.cubeStored && Shooter.getScoringLevel() == Level.None){
            Shooter.suck()
            Intake.suck()
        }else{
            Shooter.stopWheels()
        }
    }
}