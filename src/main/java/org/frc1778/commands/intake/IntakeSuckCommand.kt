package org.frc1778.commands.intake

import org.frc1778.Controls
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class IntakeSuckCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.extend()
    }

    override fun execute() {
        if(Intake.lineBreak.get()){
            Intake.suck()
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