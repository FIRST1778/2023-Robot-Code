package org.frc1778.commands.shooter

import com.github.ajalt.colormath.model.RGB
import org.frc1778.animation.BlinkAnimation
import org.frc1778.subsystems.Lights
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Robot
import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.PI
import kotlin.math.round

class ShooterShootCommand : FalconCommand(Shooter) {

    private val rebBlink = BlinkAnimation(RGB.from255(255,0,0), RGB, 4, 4)

    override fun execute() {
        val directionTowardsGrid = when (Robot.alliance) {
            DriverStation.Alliance.Red -> 0.0
            else -> PI
        }
        if(Shooter.cubeStored) {
            Shooter.shoot(
                if (directionTowardsGrid == PI * round(Gyro.odometryYaw() / PI)) {
                    Shooter.getScoringLevel().frontShooterVoltage
                } else {
                    Shooter.getScoringLevel().rearShooterVoltage
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