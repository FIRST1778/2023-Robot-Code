package org.frc1778.lib

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees

enum class GameObject(val offset : SIUnit<Radian>) {
    CONE(0.0.degrees),
    CUBE(50.0.degrees)
}