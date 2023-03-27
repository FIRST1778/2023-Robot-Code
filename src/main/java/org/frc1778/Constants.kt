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
import org.ghrobotics.lib.mathematics.units.derived.*
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
            "Top Left" to 272.3,
            "Top Right" to 17.6,
            "Bottom Right" to 48.4,
            "Bottom Left" to 273.45,
        )
        val topLeftSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Top Left Swerve"
            kDriveTalonId = 6
            kAzimuthTalonId = 5
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
            kDriveTalonId = 8
            kAzimuthTalonId = 7
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
            kDriveTalonId = 2
            kAzimuthTalonId = 1
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
            kDriveTalonId = 4
            kAzimuthTalonId = 3
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

    object ShooterConstants {
        const val ANGLE_MOTOR_GEAR_REDUCTION: Double = ((24.0 / 64.0) * (1.0 / 5.0) * (1.0 / 4.0))
        val ANGLE_MOTOR_UNIT_MODEL = NativeUnitRotationModel((42.0 * (1 / ANGLE_MOTOR_GEAR_REDUCTION)).nativeUnits)
        const val ANGLE_MOTOR_ID: Int = 16
        val shooterTab = Shuffleboard.getTab("Shooter")!!
    }

    object VisionConstants {
        const val cameraName = "The Eye of Sauron"
        val APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile)!!
        val CAMERA_ROBOT_TRANSFORM = Transform3d(
            Translation3d((-10).inches.value, 9.0.inches.value, 42.inches.value), Rotation3d(0.0, 0.0, 0.0)
        )
    }

	object LedConstants {
		const val NUM_LEDS: Int = 7 * 4
		const val NUM_LEDS_PER_SECTION: Int = 7
	}


}
//TODO Get shooter position values
enum class Level(val rearShooterPosition : SIUnit<Radian>, val rearShooterVoltage : SIUnit<Volt>, val frontShooterPosition : SIUnit<Radian>, val frontShooterVoltage : SIUnit<Volt>) {
    Top(
//        Transform3d(
//            Translation3d(39.73.inches.value, 0.0, 46.0.inches.value), Rotation3d()
//        ),
        rearShooterPosition = 230.0.degrees,
        rearShooterVoltage = 5.0.volts,
        frontShooterPosition = 150.degrees,
        frontShooterVoltage = 5.0.volts

    ),
    Middle(
//        Transform3d(
//            Translation3d((22.7.inches).value, 0.0, 34.0.inches.value), Rotation3d()
//        ),
        rearShooterPosition = 240.0.degrees,
        rearShooterVoltage = 4.0.volts,
        frontShooterPosition = 140.0.degrees,
        frontShooterVoltage = 4.0.volts
    ),
    Bottom(
//        Transform3d(
//            Translation3d(), Rotation3d()
//        ),
        rearShooterPosition = 250.0.degrees,
        rearShooterVoltage = 3.0.volts,
        frontShooterPosition = 130.0.degrees,
        frontShooterVoltage = 3.0.volts
    ),
    None(
        rearShooterPosition = 90.0.degrees,
        rearShooterVoltage = 0.0.volts,
        frontShooterPosition = 90.0.degrees,
        frontShooterVoltage = 0.0.volts
    )
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