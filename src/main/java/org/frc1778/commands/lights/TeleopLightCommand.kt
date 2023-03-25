package org.frc1778.commands.lights

import com.github.ajalt.colormath.model.RGB
import org.frc1778.subsystems.DotStar
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class TeleopLightCommand: FalconCommand(DotStar) {
    override fun initialize() {
        DotStar.animateOff()
    }

    override fun execute() {
        if(Intake.lineBreak.get()) {
            DotStar.fill(RGB.from255(255,255,0))
        }
        if(Shooter.cubeStored) {
            DotStar.fill(RGB.from255(0,255,0))
        }
        if(!Intake.lineBreak.get() && !Shooter.cubeStored) {
            DotStar.fill(RGB.from255(255, 0, 0))
        }
    }
}