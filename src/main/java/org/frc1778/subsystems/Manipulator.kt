package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.DigitalInput
import org.frc1778.Constants
import org.frc1778.lib.GameObject
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import kotlin.math.sin


object Manipulator : FalconSubsystem() {
    val lineBreak = DigitalInput(1)

    val angleMotor = falconMAX(
        Constants.ManipulatorConstants.ANGLE_MOTOR_MAIN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.ManipulatorConstants.ANGLE_MOTOR_UNIT_MODEL,
    ) {
        brakeMode = true
    }

    private const val angle_kS: Double = 5.427248300685566
    private const val angle_kA: Double = 0.9399999999999998
    private const val angle_kV: Double = 0.05952808261099462
    private val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    private var desiredAngleVelocity: Double = 0.0 // rad/s
    private var desiredAngle: SIUnit<Radian> = 0.0.radians

    val encoder = ManipulatorAbsoluteEncoder()

    var objectControlEnabled = true
    val angleControlEnabled = true

    private var gameObj : GameObject = GameObject.CUBE
    fun setGameObject(gameObject: GameObject){
        if(objectControlEnabled){
            gameObj = gameObject
        }
    }
    fun getGameObject(): GameObject{
        return gameObj
    }
    fun setDesiredAngleVelocity(velocity: Double) {
        desiredAngleVelocity = velocity
    }

    fun setDesiredAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle
    }
    fun getCurrentAngle() : SIUnit<Radian>{
        return encoder.absolutePosition
    }
    fun getDesiredAngleVelocity(): Double {
        return desiredAngleVelocity
    }
    private val angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(0.1, 4.0), VecBuilder.fill(2.0), 0.020
    )
    private val angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )

    fun angleControl(){
        if (angleControlEnabled) {
            angleLoop.setNextR(VecBuilder.fill(desiredAngle.value, desiredAngleVelocity))
            angleLoop.correct(VecBuilder.fill(getCurrentAngle().value))
            angleLoop.predict(0.020) // 20 ms

            var nextVoltage = angleLoop.getU(0)
            nextVoltage += angle_kS * sin(getCurrentAngle().value)
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            angleMotor.setVoltage(nextVoltage.volts)
        } else {
            resetDesiredAngle()
            angleMotor.setVoltage(0.0.volts)
        }
    }

    fun resetDesiredAngle() {
        desiredAngle = getCurrentAngle()
        desiredAngleVelocity = 0.0
    }

    fun initialize() {
        encoder.resetPositionRaw(0.0.nativeUnits) // TODO(Need Offset)
    }

    override fun periodic() {
        if(Arm.zeroed){
            angleControl()
        }else{
            resetDesiredAngle()
        }
    }
    override fun lateInit(){
        Constants.ManipulatorConstants.manipulatorShuffleboardTab.add(
            this
        ).withSize(3, 4)
        Constants.ManipulatorConstants.manipulatorShuffleboardTab.add(
            "Angle Encoder",
            encoder
        ).withSize(3, 4)
        Constants.ManipulatorConstants.manipulatorShuffleboardTab.add(
            "LineBreak",
            lineBreak
        ).withSize(2, 2)
    }
}