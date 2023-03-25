package org.frc1778.commands.shooter

import org.frc1778.Level
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterSuckCommand : FalconCommand(Shooter){
    override fun execute() {
        if(!Shooter.cubeStored){
            Shooter.suck()
        }else{
            Shooter.stopWheels()
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stopWheels()
    }

    override fun cancel() {
        end(true)
    }
    override fun isFinished(): Boolean {
        return Shooter.cubeStored
    }
}