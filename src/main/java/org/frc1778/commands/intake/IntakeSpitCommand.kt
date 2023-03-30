package org.frc1778.commands.intake

import org.frc1778.commands.shooter.ShooterShootCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees


class IntakeSpitCommand : FalconCommand(Intake) {
    override fun initialize() {
        Intake.spit()
        Intake.retract()
    }
    override fun execute(){
        if(Shooter.getCurrentAngle() < 95.0.degrees){
            Shooter.spit()
            Shooter.cubeStored = false
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