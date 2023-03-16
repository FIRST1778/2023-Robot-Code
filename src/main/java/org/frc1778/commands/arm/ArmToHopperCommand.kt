package org.frc1778.commands.arm

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.frc1778.subsystems.Arm
import org.frc1778.subsystems.Intake
import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.wrappers.FalconSolenoid

class ArmToHopperCommand(): FalconCommand(Arm, Intake) {
//    lateinit var command: Command
//
//    override fun initialize() {
//        command = sequential {
////            + InstantCommand({Intake.extend()})
//            + ArmExtensionCommand(0.0.meters)
//            + ArmAngleCommand(12.0.degrees)
//            + ArmExtensionCommand(7.0.inches)
//        }
//        command.schedule()
//    }
//
//    override fun end(interrupted: Boolean) {
////        Manipulator.manipulatorSol.state = FalconSolenoid.State.Reverse
//    }
//
//
//    override fun isFinished(): Boolean {
//        return command.isFinished
//    }
}