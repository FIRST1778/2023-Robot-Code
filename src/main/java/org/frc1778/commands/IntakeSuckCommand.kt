package org.frc1778.commands

import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.seconds

class IntakeSuckCommand() : FalconCommand(Intake) {
    override fun initialize() {
        Intake.suck()
        Intake.extend()
    }

    override fun cancel() {
        super.cancel()
        Intake.retract()
        Intake.stop()

    }
}