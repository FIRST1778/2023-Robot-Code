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
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
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
            SIUnit((maxSpeed.value / hypot(trackWidth / 2.0, wheelBase / 2.0)))
        val maxAngularAcceleration: SIUnit<Acceleration<Radian>> = SIUnit(maxAngularSpeed.value * (3.0))

        const val pigeonCanID: Int = 21
        private const val azimuthMotorEncoderNativeUnitsPerRotation = 21.5
        private const val driveMotorEncoderNativeUnisPerRotation = 42.0 * driveReduction

        private val swerveDriveWheelOffsets = mapOf(
            "Top Left" to 272.5,
            "Top Right" to 18.3,
            "Bottom Right" to 48.77,
            "Bottom Left" to 273.1,
        )
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
            kAzimuthEncoderHomeOffset = -Math.toRadians(swerveDriveWheelOffsets["Top Left"]!!)


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
            kAzimuthEncoderHomeOffset = -Math.toRadians(swerveDriveWheelOffsets["Top Right"]!!)


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
            kAzimuthEncoderHomeOffset = -Math.toRadians(swerveDriveWheelOffsets["Bottom Right"]!!)


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
            kAzimuthEncoderHomeOffset = -Math.toRadians(swerveDriveWheelOffsets["Bottom Left"]!!)


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
    object ManipulatorConstants{
        val ANGLE_MOTOR_MAIN_ID: Int = 15
        val ANGLE_MOTOR_UNIT_MODEL: NativeUnitRotationModel = NativeUnitRotationModel(2048.nativeUnits)
        val DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL : NativeUnitRotationModel = NativeUnitRotationModel(1.nativeUnits)
        val manipulatorShuffleboardTab = Shuffleboard.getTab("Manipulator")
    }

    object ArmConstants {

        val DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL: NativeUnitRotationModel = NativeUnitRotationModel(1.nativeUnits)
        val armShuffleboardTab = Shuffleboard.getTab("Arm")
        const val ANGLE_ENCODER_ID: Int = 23
        val ANGLE_ENCODER_UNIT_MODEL: NativeUnitModel<Radian> = NativeUnitRotationModel(4096.nativeUnits) //TODO
        val EXTENSION_MOTOR_UNIT_MODEL: NativeUnitLengthModel =
            NativeUnitLengthModel((42 / 4).nativeUnits, (1.128).inches) //TODO
        const val ANGLE_MOTOR_OTHER_ID: Int = 12
        const val EXTENSION_MOTOR_ID: Int = 13
        const val ANGLE_MOTOR_MAIN_ID: Int = 11
        val ANGLE_ENCODER_OFFSET = -(430.0 - 90.0).degrees
        val ANGLE_MOTOR_UNIT_MODEL: NativeUnitRotationModel = NativeUnitRotationModel(2048.nativeUnits) //TODO
        val ZEROED_EXTENSION_DISTANCE_READING: SIUnit<Meter> = 7.9.inches
        val AORTransform: Transform3d = Transform3d(
            Translation3d((-10.5).inches.value, 0.0, 49.5.inches.value), Rotation3d()
        )
        val ARM_EXTENSION_OFFSET = 36.5.inches
        val AllianceXOffset = 18.inches.value
    }

    object VisionConstants {
        const val cameraName = "The Eye of Sauron"
        val APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile)!!
        val CAMERA_ROBOT_TRANSFORM = Transform3d(
            Translation3d((-10).inches.value, 9.0.inches.value, 42.inches.value), Rotation3d(0.0, 0.0, 0.0)
        )
    }

    object LightConstants {
        const val numLights = 100
    }


}

enum class Level(val transform: Transform3d) {
    Top(
        Transform3d(
            Translation3d(39.73.inches.value, 0.0, 46.0.inches.value), Rotation3d()
        )
    ),
    Middle(
        Transform3d(
            Translation3d((22.7.inches).value, 0.0, 34.0.inches.value), Rotation3d()
        )
    ),
    Bottom(
        Transform3d(
            Translation3d(), Rotation3d()
        )
    );


}

enum class GamePiece(val heightOffset: Double) {
    Cube(-6.00.inches.value), Cone(5.0.inches.value)
}

//18" away from edge
enum class Station(private val redPose: Pose3d, private val bluePose: Pose3d) {
    Left(
        Pose3d(
            Translation3d(), Rotation3d()
        ), Pose3d(
            Translation3d(), Rotation3d()
        )
    ),
    Center(
        Pose3d(
            Translation3d(), Rotation3d()
        ), Pose3d(
            Translation3d(), Rotation3d()
        )
    ),
    Right(
        Pose3d(
            Translation3d(), Rotation3d()
        ), Pose3d(
            Translation3d(), Rotation3d()
        )
    );

    fun ours(): Pose3d {
        return if (Robot.alliance == DriverStation.Alliance.Red) redPose
        else bluePose
    }
}

enum class Side(val transform: Transform3d) {
    Left(
        Transform3d(
            Translation3d(
                0.0, 22.0.inches.value, 0.0
            ), Rotation3d()
        )
    ),
    Right(
        Transform3d(
            Translation3d(
                0.0, -22.inches.value, 0.0
            ), Rotation3d()
        )
    )
}