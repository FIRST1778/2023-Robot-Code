package org.frc1778.commands.shooter

import org.frc1778.Level
import org.frc1778.commands.intake.IntakeSuckCommand
import org.frc1778.commands.intake.IntakeToShooterCommand
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

class ShooterSuckCommand : FalconCommand(Shooter) {

    override fun initialize() {
        Shooter.suck()
    }

    override fun cancel() {
        super.cancel()
        end(true)
    }


    override fun end(interrupted: Boolean) {
        Shooter.stopWheels()
    }


    override fun isFinished(): Boolean {
        return Shooter.cubeStored
    }
}