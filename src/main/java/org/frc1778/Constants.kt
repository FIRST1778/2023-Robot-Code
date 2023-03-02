package org.frc1778

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
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
        private const val driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
        val maxSpeed: SIUnit<Frac<Meter, Second>> = SIUnit((5676 / 60.0) * driveReduction * 4.inches.value * PI)
        val maxAngularSpeed: SIUnit<Velocity<Radian>> =
            SIUnit(maxSpeed.value / hypot(trackWidth / 2.0, wheelBase / 2.0))

        const val pigeonCanID: Int = 21
        private const val azimuthMotorEncoderNativeUnitsPerRotation = 21.5
        private const val driveMotorEncoderNativeUnisPerRotation = 42.0 * driveReduction
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
        }


    }

    object ArmConstants {

        val EXTENSION_MOTOR_UNIT_MODEL: NativeUnitLengthModel = NativeUnitLengthModel(2048.nativeUnits, 24.milli.meters) //TODO
        const val ANGLE_MOTOR_OTHER_ID: Int = 12
        const val EXTENSION_MOTOR_ID: Int = 13
        const val ROTATION_ENCODER_CPR: Int = 2048 //TODO
        const val ANGLE_MOTOR_MAIN_ID: Int = 11
        val ANGLE_MOTOR_UNIT_MODEL: NativeUnitRotationModel = NativeUnitRotationModel(2048.nativeUnits) //TODO
        const val ZEROED_EXTENSION_DISTANCE_READING: Double = 0.0
        val AORTransform: Transform3d = Transform3d()
    }
    object VisionConstants {
        const val cameraName = "The Eye of Sauron"
        val APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile)!!
        val CAMERA_ROBOT_TRANSFORM = Transform3d(
            Translation3d(0.0, 0.0, 0.0),
            Rotation3d(0.0, 0.0, 0.0)
        )
    }

    object LightConstants {
        const val numLights = 100
    }


}

enum class Level(val transform: Transform3d) {
    Top(Transform3d()), Middle(Transform3d()), Bottom(Transform3d())
}

enum class GamePiece(val heightOffset: Double) {
    Cube(0.0), Cone(0.0)
}

enum class Station(val redPose: Pose3d, val bluePose: Pose3d) {
    Left(Pose3d(), Pose3d()), Center(Pose3d(), Pose3d()), Right(Pose3d(), Pose3d());
    fun ours(): Pose3d {
        if (Robot.alliance == DriverStation.Alliance.Red)
            return redPose
        else
            return bluePose
    }
}

enum class Side(val transform: Transform3d) {
    Left(Transform3d()), Right(Transform3d())
}