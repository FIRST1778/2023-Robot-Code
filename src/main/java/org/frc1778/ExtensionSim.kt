package org.frc1778

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DIOSim
import org.frc1778.lib.DataLogger
import org.frc1778.lib.SimulationRelativeEncoder
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.cos
import kotlin.math.abs


class ExtensionSim(initialArmPosition: SIUnit<Meter>) {
    private var linearArmPosition = initialArmPosition.value
    private var linearVelocity = 0.0
    private var linearAccel = 0.0
    private var velocityAtMotor = 0.0
    private var accelerationAtMotor = 0.0
    private var Ng = 5.0
    private var motor = DCMotor.getNEO(1)
    private var arm_mass = 3.629 // kg
    private var input = 0.0
    private var currentDraw = 0.0
    private var logger = DataLogger("extensionsim")
    private val pulleyRadius = 0.0137541 // m
    private val limitSwitchTriggerPosition = 0.0
    val maxArmPosition = 0.381.meters

    fun Ka(): Double {
        return arm_mass * pulleyRadius * motor.rOhms / (motor.KtNMPerAmp * Ng)
    }

    fun Kv(): Double {
        return Ng / pulleyRadius / motor.KvRadPerSecPerVolt // flipped from our Kw
    }

    init {
//        logger.add("velocity (m/s)") { -> linearVelocity }
        logger.add("input voltage") { -> input }
        //logger.add("current (amps)") { -> currentDraw }
        logger.add("linear position (m)") { -> linearArmPosition }
//        logger.add("Ka") { -> Ka() }
//        logger.add("Kv") { -> Kv() }
//        logger.add("Ks") { -> forcesToVoltage(0.0) }
//        logger.add("extension position") { -> Arm.getCurrentExtension().value }
        logger.add("desired position") { -> Arm.desiredExtension.value }
        logger.add("reported position") { -> Arm.getCurrentExtension().value }
        logger.add("limit switch") { -> if (Arm.limitSwitch.get()) 1.0 else 0.0 }
    }

    fun setInput(voltage: Double) {
        input = voltage
    }

    fun update(dt: Double, jointTheta: Double, encoder: SimulationRelativeEncoder, limitSwitch: DIOSim) {
        calculateAcceleration(jointTheta)
        capVelocity(dt, jointTheta)
        linearArmPosition += linearVelocity * dt
        limitSwitch.value = (linearArmPosition < limitSwitchTriggerPosition)
        encoder.setPosition(encoder.getPosition() + linearVelocity * dt)
        logger.log()
    }

    fun capVelocity(dt: Double, jointTheta: Double) {
        velocityAtMotor += accelerationAtMotor * dt
        var gravityForce = calculateGravityForce(jointTheta)
        var freeSpeed = abs(motor.getSpeed(abs(gravityForce * (pulleyRadius / Ng)), input))
        velocityAtMotor = velocityAtMotor.coerceIn(-freeSpeed, freeSpeed)
        linearVelocity = velocityAtMotor / (Ng / pulleyRadius)
    }

    fun calculateAcceleration(jointTheta: Double) {
        currentDraw = motor.getCurrent(velocityAtMotor, input)
        val torqueAtMotor: Double = motor.getTorque(currentDraw)
        var motorForce = calculateMotorForce(torqueAtMotor)
        var gravityForce = calculateGravityForce(jointTheta)
        var hardStopForce = calculateHardStopForce()
        linearAccel = (motorForce + gravityForce + hardStopForce) / arm_mass
        accelerationAtMotor = Ng * (linearAccel / pulleyRadius)
    }

    fun calculateGravityForce(jointTheta: Double): Double {
        return arm_mass * 9.81 * cos(jointTheta)
    }

    fun calculateHardStopForce(): Double {
        var k = 100/ 0.01 // 1000 N/ 1 cm
        if(linearArmPosition < -0.002) {
            return k * (0 - linearArmPosition)
        }else if(linearArmPosition > maxArmPosition.value) {
            return k * (maxArmPosition.value - linearArmPosition)
        }
        return 0.0
    }

    fun forcesToVoltage(jointTheta: Double): Double {
        return calculateGravityForce(jointTheta) * motor.rOhms * pulleyRadius / (Ng * motor.KtNMPerAmp)
    }

    fun calculateMotorForce(motorTorque: Double): Double {
        return motorTorque / (pulleyRadius / Ng)
    }

    fun getCurrentArmPosition(): Double {
        return linearArmPosition
    }

    fun getCurrentDrawAmps(): Double {
        return currentDraw
    }


}