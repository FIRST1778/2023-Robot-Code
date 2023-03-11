// TODO safety check: only let Arm move when in the right area.

package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import org.frc1778.Constants
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import kotlin.math.cos
import kotlin.math.sin

object Arm : FalconSubsystem(), Sendable {
    var limitSwitch = DigitalInput(0)

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
        outputInverted = true
        brakeMode = true
        follow(angleMotorMain)
    }

    val armEncoder = ArmJointAbsoluteEncoder().apply {
        resetPositionRaw((0.705 + .25).mod(360.0).nativeUnits)
        inverted = true
    }

    val extensionMotor = falconMAX(
        Constants.ArmConstants.EXTENSION_MOTOR_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ArmConstants.EXTENSION_MOTOR_UNIT_MODEL
    ) {
        brakeMode = true
        smartCurrentLimit = 40.amps
    }

    private const val angle_kS: Double = 1.0
    private const val angle_kA: Double = 0.03516
    private const val angle_kV: Double = 1.615
    private val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    private const val extension_kA: Double = 0.04607411898461538
    private const val extension_kV: Double = 7.213374267914612
    private const val extension_kS: Double = 0.4519871072390769
    private val extensionPlant = LinearSystemId.identifyPositionSystem(extension_kV, extension_kA)
    // This is set in the ArmTrapezoidCommand to what the profile wants, so we
    // initialize it to a dud value here to not immediately activate the arm.
    private var desiredAngle: Double = Math.toRadians(0.0)
    var desiredExtension: SIUnit<Meter> = 0.0.meters
    private var desiredExtensionVelocity: Double = 0.0 // m/s
    private var desiredAngleVelocity: Double = 0.0 // rad/s

    var angleControlEnabled: Boolean = true
    var extensionControlEnabled: Boolean = true

    var zeroed: Boolean = false

    private val angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(0.1, 4.0), VecBuilder.fill(2.0), 0.020
    )
    private val angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )
    private val extensionObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), extensionPlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val extensionController = LinearQuadraticRegulator(
        extensionPlant, VecBuilder.fill(2.0, 16.0), VecBuilder.fill(24.0), 0.020
    )
    private val extensionLoop = LinearSystemLoop(
        extensionPlant, extensionController, extensionObserver, 12.0, 0.020
    )

    val jointLogger = DataLogger("ArmJoint")

    init {
        jointLogger.add("angle (deg)", { -> getCurrentAngle().inDegrees() })
        jointLogger.add("desired angle (deg)", { -> desiredAngle.radians.inDegrees() })
        jointLogger.add("desired velocity", {-> desiredAngleVelocity.radians.inDegrees()})
        jointLogger.add("voltage", {-> angleMotorMain.voltageOutput.value} )
        jointLogger.add("current", {-> angleMotorMain.drawnCurrent.value})

    }

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
    fun getDesiredAngleVelocity(): Double{
        return desiredAngleVelocity
    }
    fun setDesiredAngleVelocity(angle: Double) {
        desiredAngleVelocity = angle
    }

    fun setDesiredAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle.value
    }

    fun getCurrentAngle(): SIUnit<Radian> {
        return armEncoder.absolutePosition
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

    fun resetDesiredAngle() {
        desiredAngle = getCurrentAngle().value
        desiredAngleVelocity = 0.0
    }

    fun resetIsZeroed() {
        resetDesiredAngle()
        extensionControlEnabled = false
        angleControlEnabled = false
        zeroed = false
    }

    fun limitSwitchHit(): Boolean {
        return limitSwitch.get()
    }

    fun doExtensionZeroingMovement() {
        extensionMotor.setVoltage((-2.0).volts)
    }

    fun zeroExtension() {
        extensionMotor.encoder.resetPosition(0.0.meters)
        zeroed = true
        desiredExtension = 0.0.meters
        extensionMotor.setVoltage(0.0.volts)
        extensionControlEnabled = true
        angleControlEnabled = true
        resetDesiredAngle()
    }

    override fun lateInit() {
        Constants.ArmConstants.armShuffleboardTab.add(
            this
        ).withSize(3, 4)
        Constants.ArmConstants.armShuffleboardTab.add(
            "Angle Encoder",
            armEncoder
        ).withSize(2, 2)

        Constants.ArmConstants.armShuffleboardTab.add(
            "Limit Switch",
            limitSwitch
        ).withSize(2, 2)
    }

    override fun periodic() {
        if (zeroed) {
            angleControl()
            extensionControl()
        }

        jointLogger.log()
    }

    override fun initSendable(builder: SendableBuilder?) {
//        builder!!.addDoubleProperty("Arm Extension Voltage", {
////            extensionMotor.voltageOutput.value
//        }, {})
//        builder.addDoubleProperty("Arm Extension Position", {
////            extensionMotor.encoder.position.inInches()
//        }, {})
        builder!!.addBooleanProperty("Is Zeroed?", {
            zeroed
        }, {})
//        builder.addDoubleProperty("Rotation Encoder", { armEncoder.absolutePosition.inDegrees() }, {})
        builder.addDoubleProperty("Arm Rotation Voltage", {
            angleMotorMain.voltageOutput.value
        }, {})

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