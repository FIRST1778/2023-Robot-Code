package org.frc1778.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.REVPhysicsSim
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Constants
import org.frc1778.Level
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import kotlin.math.PI
import kotlin.math.sin

object Wrist : FalconSubsystem() {

    val brakeModeSwitch = DigitalInput(3)

    val angleMotor = falconMAX(
        Constants.ShooterConstants.ANGLE_MOTOR_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        if (LoggedRobot.isReal()) Constants.ShooterConstants.ANGLE_MOTOR_UNIT_MODEL else NativeUnitRotationModel((2 * PI).nativeUnits),
    ) {
        brakeMode = true
        outputInverted = true
    }


    //TODO: Update to new weights
    private var feedforwardVoltage = 0.0
    private const val angle_kS: Double = 0.0
    private const val angle_kA: Double = 0.05222222222222221
    private const val angle_kV: Double = 1.071505486997903
    private val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    private var desiredAngleVelocity: Double = 0.0 // rad/s
    private var desiredAngle: SIUnit<Radian> = 0.0.radians

    private var wristRotation = 0.0

    val encoder = ShooterAbsoluteEncoder(angleMotor.canSparkMax, NativeUnitRotationModel(1.nativeUnits)).apply {
        setInverted(true)
        resetPosition((210.75 - 90.0).degrees)
    }

    val angleControlEnabled = true

    var dataLogger = DataLogger("Shooter")

    val revMotorSim = REVPhysicsSim().apply {
        addSparkMax(
            angleMotor.canSparkMax,
            DCMotor.getNEO(1).withReduction(1.0 / ((24.0 / 64.0) * (1.0 / 5.0) * (1.0 / 4.0)))
        )
    }


    init {
        angleMotor.canSparkMax.encoder.position = 0.0
        angleMotor.encoder.resetPosition(90.degrees)
        dataLogger.add("position", { getCurrentAngle().inDegrees() })
        dataLogger.add("desired position", { desiredAngle.inDegrees() })
        dataLogger.add("desired velocity", { desiredAngleVelocity })
        dataLogger.add("ss voltage", { angleLoop.getU(0) })
        dataLogger.add("ff voltage", { feedforwardVoltage })
    }

    private var scoringLevel: Level = Level.None
    private var nextLevel: Level? = null

    fun setDesiredAngleVelocity(velocity: Double) {
        desiredAngleVelocity = velocity
    }

    fun setDesiredAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle
    }

    fun getCurrentAngle(): SIUnit<Radian> {
        return if (LoggedRobot.isReal()) encoder.absolutePosition else MathUtil.inputModulus(
            angleMotor.encoder.position.value,
            0.0,
            2.0 * PI
        ).coerceIn(90.degrees.value, 300.degrees.value).radians
    }

    fun getDesiredAngleVelocity(): Double {
        return desiredAngleVelocity
    }

    private val angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(0.075, 0.075), VecBuilder.fill(.5), 0.020
    )
    private val angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )

    //TODO Get angle offset for gravity/feedforward
    val angleOffset: SIUnit<Radian> = 36.0.degrees
    private fun angleControl() {
        @Suppress("UsePropertyAccessSyntax")
        if (angleControlEnabled && ((scoringLevel != Level.None) || (nextLevel != Level.None)) || getCurrentAngle() > 92.degrees) {
            angleLoop.setNextR(VecBuilder.fill(desiredAngle.value, desiredAngleVelocity))
            angleLoop.correct(VecBuilder.fill(getCurrentAngle().value))
            angleLoop.predict(0.020) // 20 ms

            var nextVoltage = angleLoop.getU(0)

            feedforwardVoltage = angle_kS * sin(getCurrentAngle().value + angleOffset.value)
            nextVoltage += feedforwardVoltage
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            angleMotor.setVoltage(nextVoltage.volts)
        } else {
            if (LoggedRobot.isReal()) {
                resetDesiredAngle()
                angleMotor.setVoltage(0.0.volts)
            }
        }
    }

    fun setNextLevel(level: Level) {
        nextLevel = level
    }

    fun resetDesiredAngle() {
        desiredAngle = getCurrentAngle()
        desiredAngleVelocity = 0.0
    }

    fun setBrakeMode(brakeMode: Boolean) {
        angleMotor.brakeMode = brakeMode
    }

    override fun periodic() {
        try {
            angleControl()
        } catch (e: Exception) {
            DriverStation.reportError("Bad Angle ${getCurrentAngle()}", false)
        }
        wristRotation += .1
        Logger.getInstance().recordOutput(
            "Wrist Pose", Pose3d(
                Translation3d(-0.255, 0.0, 0.3175),
                Rotation3d(0.0, (-(getCurrentAngle()) + 90.0.degrees).value, 0.0)
            )
        )
        Logger.getInstance().recordOutput(
            "Wrist Angle",
            getCurrentAngle().value
        )
        Logger.getInstance().recordOutput(
            "Wrist Native Unit Rotation",
            angleMotor.canSparkMax.encoder.position
        )
    }

    override fun simulationPeriodic() {
        if(angleMotor.encoder.position < 90.degrees || angleMotor.encoder.position > 300.degrees) {
            angleMotor.setVoltage(0.0.volts)
            angleMotor.encoder.resetPosition(angleMotor.encoder.position.inDegrees().coerceIn(90.0, 300.0).degrees)
        }
        revMotorSim.run()
    }

    override fun lateInit() {
        Constants.ShooterConstants.shooterTab.add(
            "Angle Encoder",
            encoder
        ).withSize(3, 4)
        Constants.ShooterConstants.shooterTab.add(
            "Brake Mode Switch",
            brakeModeSwitch
        ).withSize(2, 2)
    }

    fun setScoringLevel(level: Level) {
        scoringLevel = level
    }

    fun getScoringLevel(): Level {
        return scoringLevel
    }


    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder!!)
        builder.addDoubleProperty("Angle Voltage", { angleMotor.voltageOutput.value }, {})
        builder.addStringProperty("Level", { scoringLevel.name }, {})
        builder.addDoubleProperty("Angle", { getCurrentAngle().inDegrees() }, {})


    }
}