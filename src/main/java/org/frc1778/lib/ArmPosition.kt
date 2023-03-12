import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.meters

enum class ArmPosition(val desiredAngle: SIUnit<Radian>, val desiredExtension : SIUnit<Meter>){
    TOP(100.0.degrees, 0.9.meters),
    MIDDLE(85.0.degrees, 0.5.meters),
    BOTTOM(60.degrees, 0.4.meters)
}