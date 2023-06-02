package org.frc1778.commands.intake

import org.frc1778.subsystems.intake.Intake
import org.frc1778.subsystems.shooter.Shooter
import org.ghrobotics.lib.commands.FalconCommand


class IntakeStopCommand : FalconCommand(Intake, Shooter) {
    override fun initialize() {
        Intake.retract()
        Intake.stop()
        Shooter.stopWheels()
    }

    override fun isFinished(): Boolean {
        return true
    }
}