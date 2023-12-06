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

        // The gear ratio for an MK4i L2 swerve module.  The value of
        // these gear ratios, multiplied together, is approximately
        // equal to 1 : 6.75.
        //
        //   https://www.chiefdelphi.com/t/sds-mk4-swerve-module/394644
        //   https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
        //   https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained
        //     SdsModuleConfigurations.java:26
        //private const val driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
        private const val driveReduction = 1.0 / 6.12

        val maxSpeed: SIUnit<Frac<Meter, Second>>
                = SIUnit((5676 / 60.0) * driveReduction * 4.inches.value * PI)
        val maxAngularSpeed: SIUnit<Velocity<Radian>>
                = SIUnit((maxSpeed.value / hypot(trackWidth / 2.0, wheelBase / 2.0)))
        val maxAngularAcceleration: SIUnit<Acceleration<Radian>>
                = SIUnit(maxAngularSpeed.value * (3.0))

        const val pigeonCanID: Int = 21

        const val azimuthMotorEncoderNativeUnitsPerRotation = 21.5
        const val driveMotorEncoderNativeUnitsPerRotation = 42.0 * driveReduction

        // See lib/SwerveModuleConstants.kt for default values and
        // comments on each member.
        //
        // Every few matches, we need to realign the swerve modules and
        // measure new kAzimuthEncoderHomeOffset values (marked ***) with
        // Shuffleboard.

        val topLeftSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Top Left Swerve"
            kDriveTalonId = 6
            kAzimuthTalonId = 5
            kCanCoderId = 10
            kAzimuthEncoderHomeOffset = -Math.toRadians(66.23)  // **********
            kInvertDrive = true
        }

        val topRightSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Top Right Swerve"
            kDriveTalonId = 8
            kAzimuthTalonId = 7
            kCanCoderId = 11
            kAzimuthEncoderHomeOffset = -Math.toRadians(252.4)  // **********
            kInvertDrive = false
        }

        val bottomRightSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Bottom Right Swerve"
            kDriveTalonId = 2
            kAzimuthTalonId = 1
            kCanCoderId = 12
            kAzimuthEncoderHomeOffset = -Math.toRadians(109.1)  // **********
            kInvertDrive = false
        }

        val bottomLeftSwerveModuleConstants = SwerveModuleConstants().apply {
            kName = "Bottom Left Swerve"
            kDriveTalonId = 4
            kAzimuthTalonId = 3
            kCanCoderId = 13
            kAzimuthEncoderHomeOffset = -Math.toRadians(250.2)  // **********
            kInvertDrive = true
        }
    }

    object ShooterConstants {
        val shooterTab = Shuffleboard.getTab("Shooter")!!
        const val ANGLE_MOTOR_GEAR_REDUCTION: Double = ((24.0 / 64.0) * (1.0 / 5.0) * (1.0 / 4.0))
        val ANGLE_MOTOR_UNIT_MODEL = NativeUnitRotationModel((42.0 * (1 / ANGLE_MOTOR_GEAR_REDUCTION)).nativeUnits)
        const val ANGLE_MOTOR_ID: Int = 16
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
    // Top, middle, and bottom row of the grid.
    Top(
        optionName = "Top",
        rearShooterPosition = 208.degrees,
        rearShooterVoltage = 4.5.volts,
        frontShooterPosition = 155.degrees,
        frontShooterVoltage = 4.5.volts
    ),
    Middle(
        optionName = "Middle",
        rearShooterPosition = 210.0.degrees,
        rearShooterVoltage = 2.5.volts,
        frontShooterPosition = 150.degrees,
        frontShooterVoltage = 3.2.volts
    ),
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
