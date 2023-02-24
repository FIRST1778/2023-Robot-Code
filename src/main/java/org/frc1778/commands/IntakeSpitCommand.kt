package org.frc1778.commands

import edu.wpi.first.wpilibj2.command.WaitCommand
import org.frc1778.subsystems.Intake
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.seconds

class IntakeSpitCommand(val duration : SIUnit<Second> = 0.0.seconds) : WaitCommand(duration.value) {
    init{
        addRequirements(Intake)
    }
    override fun initialize() {
        Intake.spit()
    }

    override fun end(interrupted: Boolean) {
        if(duration.value > 0.0){
            Intake.stop()
        }
    }
}