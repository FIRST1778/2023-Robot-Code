package org.frc1778.commands.intake

import org.frc1778.Controls
import org.frc1778.Level
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class IntakeSuckCommand() : FalconCommand(Intake) {
    private var cubeStored : Boolean = false
    override fun initialize() {
        cubeStored = Intake.cubeStored()
    }

    override fun execute() {
        if(!cubeStored){
            if(Intake.cubeStored()){
                cubeStored = true
            }else {
                Intake.extend()
                Intake.suck()
            }
        }else if(Controls.operatorControllerRed.getRawButton(3)() && Shooter.getScoringLevel() == Level.None && !Shooter.cubeStored){
            Intake.suck()
            Intake.retract()
        }else{
            Intake.stop()
            Intake.retract()
        }
    }
    override fun cancel() {
        Intake.stop()
        Intake.retract()
        super.cancel()
    }
}