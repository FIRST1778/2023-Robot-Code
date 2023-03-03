// TODO safety check: only let Arm move when in the right area.

package org.frc1778.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import org.frc1778.ArmJointSim
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.DIOSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.frc1778.Constants
import org.frc1778.ExtensionSim
import org.frc1778.lib.FalconCTREAbsoluteEncoder
import org.frc1778.lib.FalconMAXAbsoluteEncoder
import org.frc1778.lib.SimulatableCANSparkMax
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.motors.rev.falconMAX
import kotlin.math.cos
import kotlin.math.sin

object Arm : FalconSubsystem() {
    var lastNonZeroDistance: Double = 0.0 // inches
    val distanceSensor = Ultrasonic(DigitalOutput(7), DigitalInput(8)).also {
        it.isEnabled = true
    }

    var armEncoder = Encoder(1, 2, false, CounterBase.EncodingType.k4X)


    val angleMotorMain = falconMAX(
        Constants.ArmConstants.ANGLE_MOTOR_MAIN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ArmConstants.ANGLE_MOTOR_UNIT_MODEL,
    ) {
        brakeMode = true
    }
    val angleMotorOther = falconMAX(
        Constants.ArmConstants.ANGLE_MOTOR_OTHER_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ArmConstants.ANGLE_MOTOR_UNIT_MODEL,
    ) {
        brakeMode = true
        follow(angleMotorMain)
    }

    val extensionMotor = falconMAX(
        Constants.ArmConstants.EXTENSION_MOTOR_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ArmConstants.EXTENSION_MOTOR_UNIT_MODEL
    ) {
        brakeMode = true
    }


    var armEncoderReal = FalconCTREAbsoluteEncoder(
        Constants.ArmConstants.ANGLE_ENCODER_ID,
        Constants.ArmConstants.ANGLE_ENCODER_UNIT_MODEL
    )

    val angle_kS: Double = 0.3851
    val angle_kA: Double = 0.03516
    val angle_kV: Double = 1.615
    val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    val extension_kA: Double = 0.04607411898461538
    val extension_kV: Double = 7.213374267914612
    val extension_kS: Double = 0.4519871072390769
    val extensionPlant = LinearSystemId.identifyPositionSystem(extension_kV, extension_kA)

    // This is set in the ArmTrapezoidCommand to what the profile wants, so we
    // initialize it to a dud value here to not immediately activate the arm.
    var desiredAngle: Double = Math.toRadians(0.0)
    var desiredExtension: SIUnit<Meter> = 0.0.meters
    var desiredExtensionVelocity: Double = 0.0 // m/s
    var desiredAngleVelocity: Double = 0.0 // rad/s

    var angleControlEnabled: Boolean = true
    var extensionControlEnabled: Boolean = true

    var zeroed: Boolean = false

    var angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    var angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(2.0, 16.0), VecBuilder.fill(24.0), 0.020
    )
    var angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )
    var extensionObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), extensionPlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    var extensionController = LinearQuadraticRegulator(
        extensionPlant, VecBuilder.fill(2.0, 16.0), VecBuilder.fill(24.0), 0.020
    )
    var extensionLoop = LinearSystemLoop(
        extensionPlant, extensionController, extensionObserver, 12.0, 0.020
    )

    fun angleControl() {
        if (angleControlEnabled) {
            angleLoop.setNextR(VecBuilder.fill(desiredAngle, desiredAngleVelocity))
            angleLoop.correct(VecBuilder.fill(getCurrentAngle().value))
            angleLoop.predict(0.020) // 20 ms

            var nextVoltage = angleLoop.getU(0)
            nextVoltage += angle_kS * sin(getCurrentAngle().value)
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            angleMotorMain.setVoltage(nextVoltage.volts)
        } else {
            angleMotorMain.setVoltage(0.0.volts)
        }
    }

    fun setAngleVelocity(angle: Double) {
        desiredAngleVelocity = angle
    }

    fun setAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle.value
    }

    fun getCurrentAngle(): SIUnit<Radian> {
        return armEncoderReal.absolutePosition
    }

    fun extensionControl() {
        if (extensionControlEnabled) {
            extensionLoop.setNextR(VecBuilder.fill(desiredExtension.value, desiredExtensionVelocity))
            extensionLoop.correct(VecBuilder.fill(getCurrentExtension().value))
            extensionLoop.predict(0.020) // 20 ms

            var nextVoltage = extensionLoop.getU(0)
            nextVoltage += -extension_kS * cos(getCurrentAngle().value)
            if (nextVoltage > 12.0) {
                nextVoltage = 12.0
            }
            if (nextVoltage < -12.0) {
                nextVoltage = -12.0
            }

            extensionMotor.setVoltage(nextVoltage.volts)
        } else {
            extensionMotor.setVoltage(0.0.volts)
        }
    }

    fun setExtensionVelocity(velocity: Double) {
        desiredExtensionVelocity = velocity
    }

    fun setExtension(position: SIUnit<Meter>) {
        desiredExtension = position
    }

    fun getCurrentExtension(): SIUnit<Meter> {
        return extensionMotor.encoder.position
    }

    fun resetIsZeroed() {
        zeroed = false
    }

    fun limitSwitchHit(): Boolean {
        return lastNonZeroDistance <= Constants.ArmConstants.ZEROED_EXTENSION_DISTANCE_READING
    }

    fun doExtensionZeroingMovement() {
        extensionMotor.setVoltage((-5.0).volts)
    }

    fun zeroExtension() {
        extensionMotor.encoder.resetPosition(0.0.meters)
        zeroed = true
        desiredExtension = 0.0.meters
        extensionMotor.setVoltage(0.0.volts)
    }

    override fun lateInit() {
        armEncoder.samplesToAverage = 5
        armEncoder.distancePerPulse = (2 * Math.PI) / 1024
        armEncoder.setMinRate(1.0)

    }

    override fun periodic() {
        if (zeroed) {
            angleControl()
            extensionControl()
        }
    }

    override fun simulationPeriodic() {
//        extensionSim.setInput(extensionMotor.get() * RobotController.getBatteryVoltage())
//        armJointSim.setInput(angleMotorMain.get() * RobotController.getBatteryVoltage())
//
//        extensionSim.update(0.020, armJointSim.angleAtJoint(), extensionMotor.simulationEncoder, limitSwitchSim)
//        armJointSim.update(0.020, armJointSim.getMinArmLength())
//
//        armEncoderSim.distance = armJointSim.angleAtJoint()
//
//        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armJointSim.getCurrentDrawAmps(), extensionSim.getCurrentDrawAmps()))
    }

}