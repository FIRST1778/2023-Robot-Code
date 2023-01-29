package org.frc1778

import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object Constants {

    object DriveConstants {
        val maxSpeed: SIUnit<Frac<Meter, Second>> = SIUnit(8.0)
        const val wheelBase: Double = 23.5
        const val trackWidth: Double = 23.5
        const val pigeonCanID: Int = 0
        val topLeftSwerveModuleConstants = SwerveModuleConstants().apply {
             kName = "Name"
             kDriveTalonId = 2
             kAzimuthTalonId = 1
             kCanCoderId = 10

             kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
             kInvertAzimuth = false
             kInvertAzimuthSensorPhase = false
             kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
             kAzimuthNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)
             kAzimuthEncoderHomeOffset = 0.0



            // azimuth motion
             kAzimuthKp = 1.3
             kAzimuthKi = 0.05
             kAzimuthKd = 20.0
             kAzimuthKf = 0.5421
             kAzimuthIZone = 25
             kAzimuthCruiseVelocity = SIUnit(2.6) // 1698 native units
             kAzimuthAcceleration = SIUnit(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
             kAzimuthClosedLoopAllowableError = 5

            // azimuth current/voltage
             kAzimuthContinuousCurrentLimit = 30 // amps
             kAzimuthPeakCurrentLimit = 60 // amps
             kAzimuthPeakCurrentDuration = 200 // ms
             kAzimuthEnableCurrentLimit = true
             kAzimuthMaxVoltage = 10.0 // volts
             kAzimuthVoltageMeasurementFilter = 8 // # of samples in rolling average

            // azimuth measurement
             kAzimuthStatusFrame2UpdateRate = 10 // feedback for selected sensor, ms
             kAzimuthStatusFrame10UpdateRate = 10 // motion magic, ms// dt for velocity measurements, ms
             kAzimuthVelocityMeasurementWindow = 64 // # of samples in rolling average

            // general drive
             kInvertDrive = true
             kInvertDriveSensorPhase = false
             kDriveBrakeMode = true // neutral mode could change
             kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
             kDriveNativeUnitModel = NativeUnitLengthModel(4096.nativeUnits, kWheelDiameter.inches)
             kDriveDeadband = 0.01
             kDriveMaxSpeed = 10.0

            // drive current/voltage
             kDriveContinuousCurrentLimit = 30 // amps
             kDrivePeakCurrentLimit = 50 // amps
             kDrivePeakCurrentDuration = 200 // ms
             kDriveEnableCurrentLimit = true
             kDriveMaxVoltage = 10.0 // volts
             kDriveVoltageMeasurementFilter = 8 // # of samples in rolling average

            // drive measurement
             kDriveStatusFrame2UpdateRate = 15 // feedback for selected sensor, ms
             kDriveStatusFrame10UpdateRate = 200 // motion magic, ms// dt for velocity measurements, ms
             kDriveVelocityMeasurementWindow = 64 // # of samples in rolling average

        }
        val topRightSwerveModuleConstants = SwerveModuleConstants().apply {
             kName = "Name"
             kDriveTalonId = 4
             kAzimuthTalonId = 3
             kCanCoderId = 11

             kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
             kInvertAzimuth = false
             kInvertAzimuthSensorPhase = false
             kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
             kAzimuthNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)
             kAzimuthEncoderHomeOffset = 0.0



            // azimuth motion
             kAzimuthKp = 1.3
             kAzimuthKi = 0.05
             kAzimuthKd = 20.0
             kAzimuthKf = 0.5421
             kAzimuthIZone = 25
             kAzimuthCruiseVelocity = SIUnit(2.6) // 1698 native units
             kAzimuthAcceleration = SIUnit(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
             kAzimuthClosedLoopAllowableError = 5

            // azimuth current/voltage
             kAzimuthContinuousCurrentLimit = 30 // amps
             kAzimuthPeakCurrentLimit = 60 // amps
             kAzimuthPeakCurrentDuration = 200 // ms
             kAzimuthEnableCurrentLimit = true
             kAzimuthMaxVoltage = 10.0 // volts
             kAzimuthVoltageMeasurementFilter = 8 // # of samples in rolling average

            // azimuth measurement
             kAzimuthStatusFrame2UpdateRate = 10 // feedback for selected sensor, ms
             kAzimuthStatusFrame10UpdateRate = 10 // motion magic, ms// dt for velocity measurements, ms
             kAzimuthVelocityMeasurementWindow = 64 // # of samples in rolling average

            // general drive
             kInvertDrive = true
             kInvertDriveSensorPhase = false
             kDriveBrakeMode = true // neutral mode could change
             kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
             kDriveNativeUnitModel = NativeUnitLengthModel(4096.nativeUnits, kWheelDiameter.inches)
             kDriveDeadband = 0.01
             kDriveMaxSpeed = 10.0

            // drive current/voltage
             kDriveContinuousCurrentLimit = 30 // amps
             kDrivePeakCurrentLimit = 50 // amps
             kDrivePeakCurrentDuration = 200 // ms
             kDriveEnableCurrentLimit = true
             kDriveMaxVoltage = 10.0 // volts
             kDriveVoltageMeasurementFilter = 8 // # of samples in rolling average

            // drive measurement
             kDriveStatusFrame2UpdateRate = 15 // feedback for selected sensor, ms
             kDriveStatusFrame10UpdateRate = 200 // motion magic, ms// dt for velocity measurements, ms
             kDriveVelocityMeasurementWindow = 64 // # of samples in rolling average

        }
        val bottomRightSwerveModuleConstants = SwerveModuleConstants().apply {
             kName = "Name"
             kDriveTalonId = 6
             kAzimuthTalonId = 5
             kCanCoderId = 12

             kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
             kInvertAzimuth = false
             kInvertAzimuthSensorPhase = false
             kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
             kAzimuthNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)
             kAzimuthEncoderHomeOffset = 0.0



            // azimuth motion
             kAzimuthKp = 1.3
             kAzimuthKi = 0.05
             kAzimuthKd = 20.0
             kAzimuthKf = 0.5421
             kAzimuthIZone = 25
             kAzimuthCruiseVelocity = SIUnit(2.6) // 1698 native units
             kAzimuthAcceleration = SIUnit(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
             kAzimuthClosedLoopAllowableError = 5

            // azimuth current/voltage
             kAzimuthContinuousCurrentLimit = 30 // amps
             kAzimuthPeakCurrentLimit = 60 // amps
             kAzimuthPeakCurrentDuration = 200 // ms
             kAzimuthEnableCurrentLimit = true
             kAzimuthMaxVoltage = 10.0 // volts
             kAzimuthVoltageMeasurementFilter = 8 // # of samples in rolling average

            // azimuth measurement
             kAzimuthStatusFrame2UpdateRate = 10 // feedback for selected sensor, ms
             kAzimuthStatusFrame10UpdateRate = 10 // motion magic, ms// dt for velocity measurements, ms
             kAzimuthVelocityMeasurementWindow = 64 // # of samples in rolling average

            // general drive
             kInvertDrive = true
             kInvertDriveSensorPhase = false
             kDriveBrakeMode = true // neutral mode could change
             kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
             kDriveNativeUnitModel = NativeUnitLengthModel(4096.nativeUnits, kWheelDiameter.inches)
             kDriveDeadband = 0.01
             kDriveMaxSpeed = 10.0

            // drive current/voltage
             kDriveContinuousCurrentLimit = 30 // amps
             kDrivePeakCurrentLimit = 50 // amps
             kDrivePeakCurrentDuration = 200 // ms
             kDriveEnableCurrentLimit = true
             kDriveMaxVoltage = 10.0 // volts
             kDriveVoltageMeasurementFilter = 8 // # of samples in rolling average

            // drive measurement
             kDriveStatusFrame2UpdateRate = 15 // feedback for selected sensor, ms
             kDriveStatusFrame10UpdateRate = 200 // motion magic, ms// dt for velocity measurements, ms
             kDriveVelocityMeasurementWindow = 64 // # of samples in rolling average

        }
        val bottomLeftSwerveModuleConstants = SwerveModuleConstants().apply {
             kName = "Name"
             kDriveTalonId = 8
             kAzimuthTalonId = 7
             kCanCoderId = 13

             kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
             kInvertAzimuth = false
             kInvertAzimuthSensorPhase = false
             kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
             kAzimuthNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)
             kAzimuthEncoderHomeOffset = 0.0



            // azimuth motion
             kAzimuthKp = 1.3
             kAzimuthKi = 0.05
             kAzimuthKd = 20.0
             kAzimuthKf = 0.5421
             kAzimuthIZone = 25
             kAzimuthCruiseVelocity = SIUnit(2.6) // 1698 native units
             kAzimuthAcceleration = SIUnit(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
             kAzimuthClosedLoopAllowableError = 5

            // azimuth current/voltage
             kAzimuthContinuousCurrentLimit = 30 // amps
             kAzimuthPeakCurrentLimit = 60 // amps
             kAzimuthPeakCurrentDuration = 200 // ms
             kAzimuthEnableCurrentLimit = true
             kAzimuthMaxVoltage = 10.0 // volts
             kAzimuthVoltageMeasurementFilter = 8 // # of samples in rolling average

            // azimuth measurement
             kAzimuthStatusFrame2UpdateRate = 10 // feedback for selected sensor, ms
             kAzimuthStatusFrame10UpdateRate = 10 // motion magic, ms// dt for velocity measurements, ms
             kAzimuthVelocityMeasurementWindow = 64 // # of samples in rolling average

            // general drive
             kInvertDrive = true
             kInvertDriveSensorPhase = false
             kDriveBrakeMode = true // neutral mode could change
             kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
             kDriveNativeUnitModel = NativeUnitLengthModel(4096.nativeUnits, kWheelDiameter.inches)
             kDriveDeadband = 0.01
             kDriveMaxSpeed = 10.0

            // drive current/voltage
             kDriveContinuousCurrentLimit = 30 // amps
             kDrivePeakCurrentLimit = 50 // amps
             kDrivePeakCurrentDuration = 200 // ms
             kDriveEnableCurrentLimit = true
             kDriveMaxVoltage = 10.0 // volts
             kDriveVoltageMeasurementFilter = 8 // # of samples in rolling average

            // drive measurement
             kDriveStatusFrame2UpdateRate = 15 // feedback for selected sensor, ms
             kDriveStatusFrame10UpdateRate = 200 // motion magic, ms// dt for velocity measurements, ms
             kDriveVelocityMeasurementWindow = 64 // # of samples in rolling average

        }
    }
}