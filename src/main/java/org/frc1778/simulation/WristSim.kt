package org.frc1778.simulation

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import kotlin.math.cos

class WristSim (initialWristPosition: SIUnit<Radian>){
    var wristPosition = initialWristPosition
    private var velocity = 0.0.radians
    private var accel = 0.0.radians
    private var input = 0.0.volts
    private var currentDraw = 0.0.amps
    private val Ng = 54.0
    private val motor = DCMotor.getNEO(1)
    private val inertia = 0.611 // kg m^2
    private val centerOfGravity = 3.90153.inches
    private val manipulatorMass = 10.0.pounds
    private val stopTop = 100.degrees
    private val stopBottom = (-70).degrees
    private val logger = DataLogger("wristSim")

    init {
        logger.add("wrist position (deg)", {wristPosition.inDegrees()})
        logger.add("wrist velocity (deg/s)", {velocity.inDegrees()})
        logger.add("voltage", {input.value})
        logger.add("current", {getCurrentDrawAmps()})
    }

    fun Ka(): Double {
        return inertia * motor.rOhms / (motor.KtNMPerAmp * Ng)
    }

    fun Kv(): Double {
        return Ng / motor.KvRadPerSecPerVolt
    }

    fun Ks() : Double {
        return motor.rOhms * calculateGravityTorque(90.degrees, 0.degrees) / Ng / motor.KtNMPerAmp
    }

    fun setInput(voltage: SIUnit<Volt>) {
        input = voltage
    }

    fun update(dt: SIUnit<Second>, jointAngle: SIUnit<Radian>) {
        calculateAcceleration(jointAngle)
        velocity += accel * dt.inSeconds()
        wristPosition += velocity * dt.inSeconds()
        logger.log()
    }

    private fun calculateAcceleration(jointAngle: SIUnit<Radian>) {
        currentDraw = motor.getCurrent(velocity.value, input.value).amps
        var torqueAtMotor = motor.getTorque(currentDraw.value)
        var gravityTorque = calculateGravityTorque(jointAngle)
        var hardStopTorques = calculateHardStop()
        accel = ((Ng * torqueAtMotor - gravityTorque + hardStopTorques) / inertia).radians
    }

    private fun calculateHardStop(): Double {
        val k = 1
        if(wristPosition > stopTop) {
            return k * (stopTop - wristPosition).value
        }else if(wristPosition < stopBottom) {
            return k * (stopBottom - wristPosition).value
        }
        return 0.0
    }

    private fun calculateGravityTorque(jointAngle: SIUnit<Radian>, wristAngle: SIUnit<Radian> = wristPosition): Double {
        return centerOfGravity.value * manipulatorMass.value * 9.81 * cos(wristAngle.value + jointAngle.value)

    }

    fun getCurrentDrawAmps(): Double {
        return currentDraw.value
    }
}