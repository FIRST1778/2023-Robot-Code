package org.frc1778.lib

import org.frc1778.subsystems.Manipulator
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters

enum class ArmPosition(val desiredAngle: SIUnit<Radian>, val desiredExtension : SIUnit<Meter>, val desiredManipulatorAngle : SIUnit<Radian>){
    TOP(100.0.degrees, 0.9.meters, 10.0.degrees + Manipulator.getGameObject().offset), // TODO: Find measurements for manipulator
    MIDDLE(85.0.degrees, 0.5.meters, 20.0.degrees + Manipulator.getGameObject().offset),
    BOTTOM(60.degrees, 0.4.meters, 30.0.degrees + Manipulator.getGameObject().offset)
}