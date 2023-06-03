package org.frc1778.subsystems.drive

import com.gamingnight.junction.AutoLog
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.subsystems.drive.swerve.AbstractSwerveDriveInputs


@AutoLog
open class SwerveDriveInputs : AbstractSwerveDriveInputs {
    override var leftFrontVoltage: SIUnit<Volt> = 0.volts
    override var rightFrontVoltage: SIUnit<Volt> = 0.volts
    override var rightBackVoltage: SIUnit<Volt> = 0.volts
    override var leftBackVoltage: SIUnit<Volt> = 0.volts

    override var leftFrontCurrent: SIUnit<Ampere> = 0.amps
    override var rightFrontCurrent: SIUnit<Ampere> = 0.amps
    override var rightBackCurrent: SIUnit<Ampere> = 0.amps
    override var leftBackCurrent: SIUnit<Ampere> = 0.amps

    override var leftFrontPosition: SIUnit<Meter> = 0.meters
    override var rightFrontPosition: SIUnit<Meter> = 0.meters
    override var rightBackPosition: SIUnit<Meter> = 0.meters
    override var leftBackPosition: SIUnit<Meter> = 0.meters

    override var leftFrontRotation: SIUnit<Radian> = 0.radians
    override var rightFrontRotation: SIUnit<Radian> = 0.radians
    override var rightBackRotation: SIUnit<Radian> = 0.radians
    override var leftBackRotation: SIUnit<Radian> = 0.radians

    override var leftFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    override var rightFrontVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    override var rightBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds
    override var leftBackVelocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds

    override var leftFrontFeedforward: SIUnit<Volt> = 0.volts
    override var rightFrontFeedforward: SIUnit<Volt> = 0.volts
    override var rightBackFeedforward: SIUnit<Volt> = 0.volts
    override var leftBackFeedforward: SIUnit<Volt> = 0.volts

    override var gyroRaw: SIUnit<Radian> = 0.0.radians
}