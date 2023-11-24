package org.frc1778.subsystems.drive

import com.gamingnight.junction.AutoLog
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
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
    override var leftFrontDriveVoltage: SIUnit<Volt> = 0.volts
    override var rightFrontDriveVoltage: SIUnit<Volt> = 0.volts
    override var rightBackDriveVoltage: SIUnit<Volt> = 0.volts
    override var leftBackDriveVoltage: SIUnit<Volt> = 0.volts

    override var leftFrontDriveCurrent: SIUnit<Ampere> = 0.amps
    override var rightFrontDriveCurrent: SIUnit<Ampere> = 0.amps
    override var rightBackDriveCurrent: SIUnit<Ampere> = 0.amps
    override var leftBackDriveCurrent: SIUnit<Ampere> = 0.amps

    override var leftFrontSteerVoltage: SIUnit<Volt> = 0.volts
    override var rightFrontSteerVoltage: SIUnit<Volt> = 0.volts
    override var rightBackSteerVoltage: SIUnit<Volt> = 0.volts
    override var leftBackSteerVoltage: SIUnit<Volt> = 0.volts

    override var leftFrontSteerCurrent: SIUnit<Ampere> = 0.amps
    override var rightFrontSteerCurrent: SIUnit<Ampere> = 0.amps
    override var rightBackSteerCurrent: SIUnit<Ampere> = 0.amps
    override var leftBackSteerCurrent: SIUnit<Ampere> = 0.amps

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

    override var chassisSpeeds: ChassisSpeeds = ChassisSpeeds()

    override var states: List<SwerveModuleState> = listOf()
    override var desiredStates: List<SwerveModuleState> = listOf()

    override var gyroRaw: SIUnit<Radian> = 0.0.radians
}