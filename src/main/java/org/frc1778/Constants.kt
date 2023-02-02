package org.frc1778

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import kotlin.math.PI
import kotlin.math.hypot

object Constants {

    object DriveConstants {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Drive")!!
        const val wheelBase: Double = 23.5
        const val trackWidth: Double = 23.5
        const val driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
        val maxSpeed: SIUnit<Frac<Meter, Second>> = SIUnit((5676 / 60.0) * driveReduction * 4.inches.value * PI)
        val maxAngularSpeed: SIUnit<Velocity<Radian>> =
            SIUnit(maxSpeed.value / hypot(trackWidth / 2.0, wheelBase / 2.0))

        const val pigeonCanID: Int = 21
        const val azimuthMotorEncoderNativeUnitsPerRotation = 21.5
        const val driveMotorEncoderNativeUnisPerRotation = 42.0 * driveReduction
        val topLeftSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Top Left Swerve"
            kDriveTalonId = 2
            kAzimuthTalonId = 1
            kCanCoderId = 10

            kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
            kInvertAzimuth = true
            kInvertAzimuthSensorPhase = false
            kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
            kAzimuthNativeUnitModel = NativeUnitRotationModel(azimuthMotorEncoderNativeUnitsPerRotation.nativeUnits)
            kAzimuthEncoderHomeOffset = -Math.toRadians(169.54)


            // azimuth motion
            kAzimuthKp = 1.0
            kAzimuthKi = 0.0
            kAzimuthKd = 0.1
            kAzimuthKf = 0.0
            kAzimuthIZone = 0.0


            // general drive
            kInvertDrive = true
            kInvertDriveSensorPhase = false
            kDriveBrakeMode = true // neutral mode could change
            kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
            kDriveNativeUnitModel =
                NativeUnitLengthModel(driveMotorEncoderNativeUnisPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
            kDriveDeadband = 0.01
            kDriveMaxSpeed = maxSpeed.value


        }
        val topRightSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Top Right Swerve"
            kDriveTalonId = 4
            kAzimuthTalonId = 3
            kCanCoderId = 11

            kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
            kInvertAzimuth = true
            kInvertAzimuthSensorPhase = false
            kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
            kAzimuthNativeUnitModel = NativeUnitRotationModel(azimuthMotorEncoderNativeUnitsPerRotation.nativeUnits)
            kAzimuthEncoderHomeOffset = -Math.toRadians(66.09)


            // azimuth motion
            kAzimuthKp = 1.0
            kAzimuthKi = 0.0
            kAzimuthKd = 0.1
            kAzimuthKf = 0.0
            kAzimuthIZone = 0.0


            // general drive
            kInvertDrive = false
            kInvertDriveSensorPhase = false
            kDriveBrakeMode = true // neutral mode could change
            kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
            kDriveNativeUnitModel =
                NativeUnitLengthModel(driveMotorEncoderNativeUnisPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
            kDriveMaxSpeed = maxSpeed.value
        }
        val bottomRightSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Bottom Right Swerve"
            kDriveTalonId = 6
            kAzimuthTalonId = 5
            kCanCoderId = 12

            kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
            kInvertAzimuth = true
            kInvertAzimuthSensorPhase = false
            kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
            kAzimuthNativeUnitModel = NativeUnitRotationModel(azimuthMotorEncoderNativeUnitsPerRotation.nativeUnits)
            kAzimuthEncoderHomeOffset = -Math.toRadians(175.51 + 180.0)


            // azimuth motion
            kAzimuthKp = 1.0
            kAzimuthKi = 0.0
            kAzimuthKd = 0.1
            kAzimuthKf = 0.0
            kAzimuthIZone = 0.0


            // general drive
            kInvertDrive = false
            kInvertDriveSensorPhase = false
            kDriveBrakeMode = true // neutral mode could change
            kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
            kDriveNativeUnitModel =
                NativeUnitLengthModel(driveMotorEncoderNativeUnisPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
            kDriveDeadband = 0.01
            kDriveMaxSpeed = maxSpeed.value


        }
        val bottomLeftSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Bottom Left Swerve"
            kDriveTalonId = 8
            kAzimuthTalonId = 7
            kCanCoderId = 13

            kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

            // general azimuth
            kInvertAzimuth = true
            kInvertAzimuthSensorPhase = false
            kAzimuthBrakeMode = true // neutral mode could change
            //         kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
            kAzimuthNativeUnitModel = NativeUnitRotationModel(azimuthMotorEncoderNativeUnitsPerRotation.nativeUnits)
            kAzimuthEncoderHomeOffset = -Math.toRadians(252.77)


            // azimuth motion
            kAzimuthKp = 1.0
            kAzimuthKi = 0.0
            kAzimuthKd = 0.1
            kAzimuthKf = 0.0
            kAzimuthIZone = 0.0


            // general drive
            kInvertDrive = true
            kInvertDriveSensorPhase = false
            kDriveBrakeMode = true // neutral mode could change
            kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
            kDriveNativeUnitModel =
                NativeUnitLengthModel(driveMotorEncoderNativeUnisPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
            kDriveDeadband = 0.01
            kDriveMaxSpeed = maxSpeed.value


        }


    }

    object VisionConstants {
        val cameraName = "Default"
        val APIRL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile)
        val CAMERA_ROBOT_TRANSFORM = Transform3d(
            Translation3d(0.0, 0.0, 0.0),
            Rotation3d(0.0, 0.0, 0.0)
        )
    }
}