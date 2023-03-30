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
import org.frc1778.Level
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import kotlin.math.sin

object Shooter : FalconSubsystem(), Sendable {


    private val nativeModel: NativeUnitRotationModel = NativeUnitRotationModel(42.nativeUnits)
    private val parentShooterMotor = falconMAX(11, CANSparkMaxLowLevel.MotorType.kBrushless, nativeModel) {
        outputInverted = false
        brakeMode = true

    }

    private val childShooterMotor = falconMAX(12, CANSparkMaxLowLevel.MotorType.kBrushless, nativeModel) {
        brakeMode = true
        outputInverted = false
        canSparkMax.follow(
            parentShooterMotor.canSparkMax,
            true
        )
    }

    //TODO Get DIO
    val limitSwitch = DigitalInput(2)
    val brakeModeSwitch = DigitalInput(3)

    val angleMotor = falconMAX(
        Constants.ShooterConstants.ANGLE_MOTOR_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ShooterConstants.ANGLE_MOTOR_UNIT_MODEL,
    ) {
        brakeMode = true
        outputInverted = true
    }


    var shooterSetVoltage = 0.0.volts

    //TODO: Update to new weights
    private var feedforwardVoltage = 0.0
    private const val angle_kS: Double = 0.0
    private const val angle_kA: Double = 0.05222222222222221
    private const val angle_kV: Double = 1.071505486997903
    private val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    private var desiredAngleVelocity: Double = 0.0 // rad/s
    private var desiredAngle: SIUnit<Radian> = 0.0.radians

    val encoder = ShooterAbsoluteEncoder(angleMotor.canSparkMax, NativeUnitRotationModel(1.nativeUnits)).apply {
        setInverted(true)
        resetPosition((210.75 - 90.0).degrees)
    }

    val angleControlEnabled = true

    var dataLogger = DataLogger("Shooter")

    var cubeStored = false

    init {
        dataLogger.add("position", { getCurrentAngle().inDegrees() })
        dataLogger.add("desired position", { desiredAngle.inDegrees() })
        dataLogger.add("desired velocity", { desiredAngleVelocity })
        dataLogger.add("ss voltage", { angleLoop.getU(0) })
        dataLogger.add("ff voltage", { feedforwardVoltage })
    }

    private var scoringLevel: Level = Level.None
    private var nextLevel : Level? = null

    fun setDesiredAngleVelocity(velocity: Double) {
        desiredAngleVelocity = velocity
    }

    fun setDesiredAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle
    }

    fun getCurrentAngle(): SIUnit<Radian> {
        return encoder.absolutePosition
    }

    fun getDesiredAngleVelocity(): Double {
        return desiredAngleVelocity
    }

    private val angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(0.1, 0.15), VecBuilder.fill(.55), 0.020
    )
    private val angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )
    //TODO Get angle offset for gravity/feedforward
    val angleOffset : SIUnit<Radian> = 36.0.degrees
    fun angleControl() {
        if (angleControlEnabled && ((scoringLevel != Level.None) || (nextLevel != Level.None ))){
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
            resetDesiredAngle()
            angleMotor.setVoltage(0.0.volts)
        }
    }
    fun setNextLevel(level : Level){
        nextLevel = level
    }

    fun resetDesiredAngle() {
        desiredAngle = getCurrentAngle()
        desiredAngleVelocity = 0.0
    }

    fun setBrakeMode(brakeMode: Boolean) {
        angleMotor.brakeMode = brakeMode
    }

    fun shoot(voltage : SIUnit<Volt>){
        shooterSetVoltage = voltage
        parentShooterMotor.setVoltage(voltage)
    }
    //TODO Get best intake speed
    fun suck(){
       parentShooterMotor.setVoltage((-1.5).volts)
    }

    override fun periodic() {

        angleControl()

        dataLogger.log()
        if(!limitSwitch.get()){
            cubeStored = true
        }
    }
    override fun lateInit() {
        Constants.ShooterConstants.shooterTab.add(
            this
        ).withSize(3, 4)
        Constants.ShooterConstants.shooterTab.add(
            "Angle Encoder",
            encoder
        ).withSize(3, 4)
        Constants.ShooterConstants.shooterTab.add(
            "Limit Switch",
            limitSwitch
        ).withSize(2, 2)
        Constants.ShooterConstants.shooterTab.add(
            "Brake Mode Switch",
            brakeModeSwitch
        ).withSize(2, 2)
    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addDoubleProperty("Shooter Voltage", {
            parentShooterMotor.voltageOutput.value
        }, {})
        builder.addDoubleProperty("Angle Voltage", { angleMotor.voltageOutput.value}, {})
        builder.addBooleanProperty("Shooter Loaded", { cubeStored}, {})
        builder.addStringProperty("Level", { scoringLevel.name}, {})
        builder.addDoubleProperty("Shooter Set Voltage", { shooterSetVoltage.value}, {})
        builder.addDoubleProperty("Angle", { getCurrentAngle().inDegrees() }, {})
    }

    fun stopWheels() {
        parentShooterMotor.setVoltage(0.0.volts)
    }

    fun setScoringLevel(level: Level) {
        scoringLevel = level
    }
    fun getScoringLevel(): Level{
        return scoringLevel
    }

    fun spit() {
        parentShooterMotor.setVoltage(0.75.volts)
    }
}