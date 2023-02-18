package org.frc1778

import edu.wpi.first.math.system.plant.DCMotor
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import kotlin.math.cos

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

    init {
        logger.add("velocity (m/s)", { -> linearVelocity })
        logger.add("input voltage", { -> input })
        logger.add("current (amps)", { -> currentDraw })
        logger.add("linear position (m)", { -> linearArmPosition})
    }
    fun setInput(voltage: Double){
        input = voltage
    }
    fun update(dt: Double, jointTheta : Double){
        calculateAcceleration(jointTheta)
        capVelocity(dt)
        linearArmPosition += linearVelocity * dt
        logger.log()
    }
    fun capVelocity(dt : Double){
        velocityAtMotor += accelerationAtMotor * dt
        if (velocityAtMotor > getMotorSpeed(0.0, input)){
            velocityAtMotor =getMotorSpeed(0.0, input)
        }
        if(velocityAtMotor < -getMotorSpeed(0.0, input)){
            velocityAtMotor = -getMotorSpeed(0.0, input)
        }
        linearVelocity = velocityAtMotor / (Ng / pulleyRadius)
    }

    fun getMotorSpeed(torqueNm: Double, voltageInputVolts: Double): Double {
        return voltageInputVolts * motor.KvRadPerSecPerVolt - 1.0 / motor.KtNMPerAmp * torqueNm * motor.rOhms * motor.KvRadPerSecPerVolt
    }
    fun calculateAcceleration(jointTheta: Double){
        currentDraw = motor.getCurrent(velocityAtMotor, input)
        val torqueAtMotor: Double = motor.getTorque(currentDraw)
        var motorForce = calculateMotorForce(torqueAtMotor)
        var gravityForce = calculateGravityForce(jointTheta)
        linearAccel = (motorForce + gravityForce) / arm_mass
        accelerationAtMotor = Ng * (linearAccel / pulleyRadius)
    }
    fun calculateGravityForce(jointTheta: Double) : Double{
        return arm_mass * 9.81 * cos(jointTheta)
    }
    fun forcesToVoltage(jointTheta: Double) : Double{
        return (calculateGravityForce(jointTheta) * (motor.rOhms * pulleyRadius) / (Ng* motor.KtNMPerAmp))
    }
    fun calculateMotorForce(motorTorque : Double) : Double{
        return motorTorque / (pulleyRadius / Ng)
    }

    fun getCurrentArmPosition() : Double{
        return linearArmPosition
    }

    fun getCurrentDrawAmps(): Double {
        return currentDraw
    }



}