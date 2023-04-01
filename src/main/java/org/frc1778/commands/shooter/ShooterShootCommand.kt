package org.frc1778.commands.shooter

import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.Shooter
import org.frc1778.subsystems.Wrist
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees

class ShooterShootCommand : FalconCommand(Shooter, Intake) {



    override fun execute() {

        Shooter.shoot(
            if (Gyro.direction180() == Gyro.directionTowardsGrid()) {
                Wrist.getScoringLevel().frontShooterVoltage
            } else {
                Wrist.getScoringLevel().rearShooterVoltage
            }
        )
        if (Wrist.getCurrentAngle() < 95.degrees) {
            Intake.spit()
        }

        if (!Shooter.cubeStored) {
            Lights.setAnimation(Lights.rebBlink)
            Lights.animateOn()
        }
    }

    override fun cancel() {
        super.cancel()
        Shooter.stopWheels()
        Shooter.cubeStored = false
    }
}
