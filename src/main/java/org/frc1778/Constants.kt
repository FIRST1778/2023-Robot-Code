package org.frc1778

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.units.Frac
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches
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

	/* The gear ratio for an MK4i L2 swerve module.
	 * A table with the gear ratios can be found here:
	 *
	 *   https://www.chiefdelphi.com/t/sds-mk4-swerve-module/394644
	 *
	 * The value of these gear ratios, multiplied together, is
	 * approximately equal to 1 : 6.75; this simpler number can be
	 * found here:
	 *
	 *   https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
	 *
	 * The official (but unmaintained) SwerveDriveSpecialties
	 * repository with these values is here:
	 *
	 *   https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java#L26C7-L26C7
	 */
        private const val driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)

        val maxSpeed: SIUnit<Frac<Meter, Second>> = SIUnit((5676 / 60.0) * driveReduction * 4.inches.value * PI)
        val maxAngularSpeed: SIUnit<Velocity<Radian>> =
            SIUnit((maxSpeed.value / hypot(trackWidth / 2.0, wheelBase / 2.0)))
        val maxAngularAcceleration: SIUnit<Acceleration<Radian>> = SIUnit(maxAngularSpeed.value * (3.0))

        const val pigeonCanID: Int = 21
        private const val azimuthMotorEncoderNativeUnitsPerRotation = 21.5
        private const val driveMotorEncoderNativeUnitsPerRotation = 42.0 * driveReduction

        private val swerveDriveWheelOffsets = mapOf(
            "Top Left" to 66.5,
            "Top Right" to 249.4,
            "Bottom Right" to 110.35,
            "Bottom Left" to 255.45,
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
                NativeUnitLengthModel(driveMotorEncoderNativeUnitsPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
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
                NativeUnitLengthModel(driveMotorEncoderNativeUnitsPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
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
                NativeUnitLengthModel(driveMotorEncoderNativeUnitsPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
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
                NativeUnitLengthModel(driveMotorEncoderNativeUnitsPerRotation.nativeUnits, (kWheelDiameter / 2).inches)
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
        val MORSE_CODE = arrayOf(
            ".-", "-...", "-.-.", "-..", // A-D
            ".", "..-.", "--.", "....", // E-H
            "..", ".---", "-.-", ".-..", // I-L
            "--", "-.", "---", ".--.", // M-P
            "--.-", ".-.", "...", "-", // Q-T
            "..-", "...-", ".--", "-..-", // U-X
            "-.--", "--.." // Y-Z
        )
        const val TICKS_PER_DIT = 8  // 8 * 20 ms = 160 ms per dit
	}


}

enum class Level(
    val optionName: String,
    val rearShooterPosition: SIUnit<Radian>,
    val rearShooterVoltage: SIUnit<Volt>,
    val frontShooterPosition: SIUnit<Radian>,
    val frontShooterVoltage: SIUnit<Volt>
) {
    //Shoot for the Top
    Top(
        optionName = "Top",
        rearShooterPosition = 207.5.degrees,
        rearShooterVoltage = 3.8.volts,
        frontShooterPosition = 155.degrees,
        frontShooterVoltage = 4.5.volts

    ),
    //Shoot to the Middle
    Middle(
        optionName = "Middle",
        rearShooterPosition = 210.0.degrees,
        rearShooterVoltage = 2.5.volts,
        frontShooterPosition = 150.degrees,
        frontShooterVoltage = 3.2.volts
    ),
    //Shoot to the Bottom
    Bottom(
        optionName = "Bottom",
        rearShooterPosition = 275.0.degrees,
        rearShooterVoltage = 1.5.volts,
        frontShooterPosition = 90.0.degrees,
        frontShooterVoltage = 3.0.volts
    ),
    //Stow Shooter
    None(
        optionName = "Hopper",
        rearShooterPosition = 90.0.degrees,
        rearShooterVoltage = 1.5.volts,
        frontShooterPosition = 90.0.degrees,
        frontShooterVoltage = 1.5.volts
    ),
    //Shoot from Across Charge Station
    THREE_POINT(
        optionName = "3 Point",
        rearShooterPosition = 225.0.degrees,
        rearShooterVoltage = 10.0.volts,
        frontShooterPosition = 150.0.degrees,
        frontShooterVoltage = 10.0.volts
    ),
    //Shoot From atop Charge Station
    CHARGE_STATION(
        optionName = "Don't Choose ME",
        rearShooterPosition = 225.0.degrees,
        rearShooterVoltage = 10.0.volts,
        frontShooterPosition = 150.0.degrees,
        frontShooterVoltage = 10.0.volts
    )
}
