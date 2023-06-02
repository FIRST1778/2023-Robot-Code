package org.frc1778.commands.intake

import org.frc1778.subsystems.intake.Intake
import org.ghrobotics.lib.commands.FalconCommand

class IntakeSuckCommand : FalconCommand(Intake) {
    override fun initialize() {
        if(!Intake.cubeStored()) {
            Intake.extend()
            Intake.suck()
        }
    }

    override fun execute() {
        //Ensures the intake is up if the shooter is not in hopper
        Intake.extend()
    }


    override fun cancel() {
        super.cancel()
        end(true)
    }

    override fun end(interrupted: Boolean) {
        Intake.retract()
        Intake.stop()
    }

    override fun isFinished(): Boolean {
        return Intake.cubeStored()
    }
}