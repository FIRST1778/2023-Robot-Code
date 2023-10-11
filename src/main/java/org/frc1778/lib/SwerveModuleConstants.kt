package org.frc1778.lib

import org.frc1778.Constants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class SwerveModuleConstants {
    var kName = "Name"
    var kDriveTalonId = -1
    var kAzimuthTalonId = -1
    var kCanCoderId = -1

    var kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

    // Azimuth constants

    var kInvertAzimuth = true
    var kInvertAzimuthSensorPhase = false
    var kAzimuthBrakeMode = true // neutral mode could change
    //        var kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
    var kAzimuthNativeUnitModel = NativeUnitRotationModel(Constants.DriveConstants
                    .azimuthMotorEncoderNativeUnitsPerRotation.nativeUnits)
    var kAzimuthEncoderHomeOffset = 0.0

    // https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
    var kAzimuthKp = 1.0  // proportional
    var kAzimuthKi = 0.0  // integral
    var kAzimuthKd = 0.1  // derivative
    var kAzimuthKf = 0.0  // feed-forward
    var kAzimuthIZone = 0.0  // integration zone

    var kAzimuthCruiseVelocity = SIUnit<Velocity<Radian>>(2.6) // 1698 native units
    var kAzimuthAcceleration = SIUnit<Acceleration<Radian>>(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
    var kAzimuthClosedLoopAllowableError = 5

    // azimuth current/voltage
    var kAzimuthContinuousCurrentLimit = 30 // amps
    var kAzimuthPeakCurrentLimit = 60 // amps
    var kAzimuthPeakCurrentDuration = 200 // ms
    var kAzimuthEnableCurrentLimit = true
    var kAzimuthMaxVoltage = 10.0 // volts
    var kAzimuthVoltageMeasurementFilter = 8 // # of samples in rolling average

    // azimuth measurement
    var kAzimuthStatusFrame2UpdateRate = 10 // feedback for selected sensor, ms
    var kAzimuthStatusFrame10UpdateRate = 10 // motion magic, ms// dt for velocity measurements, ms
    var kAzimuthVelocityMeasurementWindow = 64 // # of samples in rolling average

    // general drive
    var kInvertDrive = true
    var kInvertDriveSensorPhase = false
    var kDriveBrakeMode = true // neutral mode could change
    var kWheelDiameter = 4.0 // Could tune for each individual wheel, but we don't
    var kDriveNativeUnitModel = NativeUnitLengthModel(Constants.DriveConstants
                .driveMotorEncoderNativeUnitsPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
    var kDriveDeadband = 0.01
    // var kDriveMaxSpeed = 10.0
    var kDriveMaxSpeed = Constants.DriveConstants.maxSpeed.value

    // drive current/voltage
    var kDriveContinuousCurrentLimit = 30 // amps
    var kDrivePeakCurrentLimit = 50 // amps
    var kDrivePeakCurrentDuration = 200 // ms
    var kDriveEnableCurrentLimit = true
    var kDriveMaxVoltage = 11.0 // volts
    var kDriveVoltageMeasurementFilter = 8 // # of samples in rolling average

    // drive measurement
    var kDriveStatusFrame2UpdateRate = 15 // feedback for selected sensor, ms
    var kDriveStatusFrame10UpdateRate = 200 // motion magic, ms// dt for velocity measurements, ms
    var kDriveVelocityMeasurementWindow = 64 // # of samples in rolling average
}
