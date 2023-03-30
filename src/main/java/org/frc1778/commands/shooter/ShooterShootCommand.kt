package org.frc1778.commands.shooter

import com.github.ajalt.colormath.model.RGB
import org.frc1778.animation.BlinkAnimation
import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.frc1778.subsystems.Wrist
import org.ghrobotics.lib.commands.FalconCommand

class ShooterShootCommand : FalconCommand(Shooter) {

    private val rebBlink = BlinkAnimation(RGB.from255(255,0,0), RGB, 4, 4)

    override fun execute() {
        if(Shooter.cubeStored) {
            Shooter.shoot(
                if (Gyro.direction180() == Gyro.directionTowardsGrid()) {
                    Wrist.getScoringLevel().frontShooterVoltage
                } else {
                    Wrist.getScoringLevel().rearShooterVoltage
                }
            )
        }else{
            Lights.setAnimation(rebBlink)
            Lights.animateOn()
        }
    }

    override fun cancel() {
        super.cancel()
        Shooter.stopWheels()
        Shooter.cubeStored = false
    }
}
