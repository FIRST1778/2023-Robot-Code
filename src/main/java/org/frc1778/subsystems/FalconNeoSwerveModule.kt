package org.frc1778.subsystems

import com.ctre.phoenix.sensors.CANCoder
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.frc1778.lib.AbstractFalconSwerveModule
import org.frc1778.lib.FalconCanCoder
import org.frc1778.lib.SwerveModuleConstants
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.motors.AbstractFalconMotor
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.rev.falconMAX

class FalconNeoSwerveModule(private val swerveModuleConstants: SwerveModuleConstants) : AbstractFalconSwerveModule {
    override var driveMotor: AbstractFalconMotor<Meter> = with(swerveModuleConstants) {
        falconMAX(
            kDriveTalonId,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            kDriveNativeUnitModel
        ) {
            outputInverted = kInvertDrive
            brakeMode = kDriveBrakeMode
            encoder.canEncoder.inverted = kInvertDriveSensorPhase
        }
    }
    override var turnMotor: AbstractFalconMotor<Radian> = with(swerveModuleConstants) {
        falconMAX(
            kAzimuthTalonId,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            kAzimuthNativeUnitModel
        ) {
            outputInverted = kInvertAzimuth
            brakeMode = kAzimuthBrakeMode
            encoder.canEncoder.inverted = brakeMode
        }
    }
    override var encoder: FalconEncoder<Radian> = FalconCanCoder(swerveModuleConstants.kCanCoderId, swerveModuleConstants.kCanCoderNativeUnitModel)

    override fun setControls(speed: Double, azimuth: Rotation2d) {
        TODO("Not yet implemented")
    }

    override fun setState(state: SwerveModuleState, arbitraryFeedForward: SIUnit<Volt>) {
        TODO("Not yet implemented")
    }

    override fun setPositions(positions: SwerveModulePosition, arbitraryFeedForward: SIUnit<Volt>) {
        TODO("Not yet implemented")
    }

    override fun resetAngle(angle: SIUnit<Radian>) {
        TODO("Not yet implemented")
    }

    override fun resetDriveEncoder(position: SIUnit<Meter>) {
        TODO("Not yet implemented")
    }

    override fun reset() {
        TODO("Not yet implemented")
    }

    override fun state(): SwerveModuleState {
        TODO("Not yet implemented")
    }

    override fun swervePosition(): SwerveModulePosition {
        TODO("Not yet implemented")
    }

    override fun setNeutral() {
        driveMotor.setNeutral()
    }

    override val voltageOutput: SIUnit<Volt> = driveMotor.voltageOutput
    override val drawnCurrent: SIUnit<Ampere> = driveMotor.drawnCurrent
    override val drivePosition: SIUnit<Meter> = driveMotor.encoder.position
    override val driveVelocity: SIUnit<Velocity<Meter>> = driveMotor.encoder.velocity
    override val anglePosition: SIUnit<Radian> = encoder.position
}