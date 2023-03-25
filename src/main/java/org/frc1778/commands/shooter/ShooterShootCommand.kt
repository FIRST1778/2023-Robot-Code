package org.frc1778.commands.shooter

import com.github.ajalt.colormath.model.RGB
import org.frc1778.animation.BlinkAnimation
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class ShooterShootCommand : FalconCommand(Shooter) {

    private val rebBlink = BlinkAnimation(RGB.from255(255,0,0), RGB, 4, 4)


    override fun initialize() {
        if(Shooter.cubeStored) {
            Shooter.shoot(Shooter.getScoringLevel().shooterVoltage)
            Shooter.cubeStored = false
        } else {
            Lights.setAnimation(rebBlink)
            Lights.animateOn()
        }
    }

    override fun cancel() {
        super.cancel()
        Shooter.stopWheels()
    }
}