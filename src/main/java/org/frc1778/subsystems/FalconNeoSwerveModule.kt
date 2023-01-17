package org.frc1778.subsystems

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
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.motors.rev.falconMAX
import kotlin.math.PI

class FalconNeoSwerveModule(private val swerveModuleConstants: SwerveModuleConstants) : AbstractFalconSwerveModule<FalconMAX<Meter>, FalconMAX<Radian>> {
    private var resetIteration: Int = 0
    private var referenceAngle: Double = 0.0

    override var driveMotor = with(swerveModuleConstants) {
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
    override var turnMotor = with(swerveModuleConstants) {
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

    private var angleController = with(swerveModuleConstants) {
        turnMotor.controller.apply{
            ff = kAzimuthKf
            p = kAzimuthKp
            i = kAzimuthKi
            d = kAzimuthKd
            setFeedbackDevice(turnMotor.canSparkMax.encoder)
        }

    }

    override fun setControls(speed: Double, azimuth: Rotation2d) {
        TODO("Not yet implemented")
    }

    override fun setState(state: SwerveModuleState, arbitraryFeedForward: SIUnit<Volt>) {
        var setAngle = state.angle.radians % (2*Math.PI)
        var voltage = state.speedMetersPerSecond/ swerveModuleConstants.kDriveMaxSpeed
        if( setAngle < 0.0) setAngle += 2.0 * Math.PI


        var diff = setAngle - stateAngle()
        if(diff >= PI) {
            setAngle -= 2.0 * PI
        } else if(diff < -PI) {
            setAngle += 2.0 * PI
        }
        diff = setAngle - stateAngle()
        if(diff > PI/2.0 || diff < -PI/2.0) {
            setAngle += PI
            voltage *= -1
        }

        setAngle %= 2.0 * PI
        if(setAngle < 0.0) setAngle += 2.0 * PI

        setVoltage(voltage)


    }

    fun stateAngle(): Double {
        var motorAngle = turnMotor.encoder.position.value
        motorAngle %= 2.0 * PI
        if(motorAngle < 0.0) motorAngle += 2.0 * PI
        return motorAngle
    }



    override fun setPosition(position: SwerveModulePosition, arbitraryFeedForward: SIUnit<Volt>) {
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

    override fun setAngle(angle: Double) {
        var currentAngleRadians = turnMotor.encoder.position.value

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (turnMotor.encoder.velocity.absoluteValue.value < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0
                val absoluteAngle: Double = encoder.position.value
                turnMotor.encoder.resetPosition(absoluteAngle.radians)
                currentAngleRadians = absoluteAngle
            }
        } else {
            resetIteration = 0
        }

        var currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI)
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        var adjustedReferenceAngleRadians: Double = referenceAngle + currentAngleRadians - currentAngleRadiansMod
        if (referenceAngle - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI
        } else if (referenceAngle - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI
        }

        this.referenceAngle = referenceAngle

    }

    override fun setVoltage(voltage: Double) {
        driveMotor.setVoltage(voltage.volts)
    }


    override val voltageOutput: SIUnit<Volt> = driveMotor.voltageOutput
    override val drawnCurrent: SIUnit<Ampere> = driveMotor.drawnCurrent
    override val drivePosition: SIUnit<Meter> = driveMotor.encoder.position
    override val driveVelocity: SIUnit<Velocity<Meter>> = driveMotor.encoder.velocity
    override val anglePosition: SIUnit<Radian> = encoder.position

    companion object {
        private const  val ENCODER_RESET_ITERATIONS = 500
        private val ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5)
    }
}