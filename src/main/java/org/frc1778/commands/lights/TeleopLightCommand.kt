package org.frc1778.commands.lights

import com.github.ajalt.colormath.model.RGB
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Robot
import org.frc1778.subsystems.intake.Intake
import org.frc1778.subsystems.Lights
import org.frc1778.subsystems.shooter.Shooter
import org.ghrobotics.lib.commands.FalconCommand

class TeleopLightCommand: FalconCommand(Lights) {
    override fun initialize() {
        Lights.animateOff()
    }

    override fun execute() {
        if(Intake.cubeStored()) {
            Lights.fill(RGB.from255(255,190,0)) // Orange
        }
        if(Shooter.cubeStored) {
            Lights.fill(RGB.from255(0,255,0)) // Green
        }
        if(!Intake.cubeStored() && !Shooter.cubeStored) {
            if(Robot.alliance == DriverStation.Alliance.Blue){
                Lights.fill(RGB.from255(0,0,255)) // Blue
            }else{
                Lights.fill(RGB.from255(255, 0, 0)) // Red
            }
        }
    }
}