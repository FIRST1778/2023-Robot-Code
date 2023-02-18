package org.frc1778

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.Timer
import java.io.File
import java.io.PrintWriter
import org.frc1778.lib.DataLogger

class ArmJointSim(initialJointAngle: Double){
    private var logger = DataLogger("armjointsim")
    init {
        logger.add("velocity (rad/s)", { -> velocity })
        logger.add("angle (deg)", { -> Math.toDegrees(joint_theta) })
        logger.add("input voltage", { -> input })
        logger.add("current (amps)", { -> currentDraw })
        logger.add("accel (rad/s^2)", { -> accel})
    }

    private var velocity = 0.0
    private var accel = 0.0
    private var joint_theta = initialJointAngle
    private var Ng = 80
    private var motor = DCMotor.getNEO(1)
    private var motor_count = 2
    private var arm_mass = 4.536 // kg
    private var spring_spur_length = 0.0762 // m
    private var spring_h_length = 0.381 // m
    private var spring_constant = 0 // N / m 845
    private var input = 0.0
    private var currentDraw = 0.0
    private var min_arm_length = 0.7 // m
    private var max_arm_length = 2.0 // m
    private var min_arm_inertia = 1.219 // m^2 kg
    private var max_arm_inertia = 4.07 // m^2 kg
    private var min_arm_cog = 0.3 // m
    private var max_arm_cog = 0.6 // m
    fun setInput(voltage: Double){
        input = voltage
    }
    fun update(dt: Double, arm_length: Double){
        calculateAcceleration(arm_length)
        capVelocity(dt)
        joint_theta += velocityAtJoint() * dt
        val time: Double = Timer.getFPGATimestamp()
        logger.log()
    }
    fun capVelocity(dt : Double){
        velocity += accel * dt
        if (velocity > motor.freeSpeedRadPerSec){
            velocity = motor.freeSpeedRadPerSec
        }
        if(velocity < -motor.freeSpeedRadPerSec){
            velocity = -motor.freeSpeedRadPerSec
        }
    }
    fun calculateAcceleration(arm_length: Double){
        currentDraw = motor.getCurrent(velocity, input)
        val torqueAtMotor: Double = motor.getTorque(currentDraw)
        val inertia: Double = calculateInertia(arm_length)
        accel = Ng * (Ng * motor_count * torqueAtMotor + springTorque() - armTorqueGravity(arm_length)) / inertia
    }
    fun torquesToVoltage(arm_length: Double) : Double{
        return (armTorqueGravity(arm_length) - springTorque()) * (motor.rOhms / (motor_count * Ng* motor.KtNMPerAmp))
    }
    fun armTorqueGravity(arm_length : Double): Double{
        return 9.81 * arm_mass * calculateCog(arm_length) * Math.sin(joint_theta)
    }
    fun springTorque(): Double {
        return spring_constant * spring_spur_length * spring_h_length * Math.sin(joint_theta)
    }

    fun velocityAtJoint(): Double {
        return velocity / Ng
    }
    fun calculateInertia(arm_length: Double): Double {
        return min_arm_inertia + (max_arm_inertia - min_arm_inertia) / (max_arm_length - min_arm_length) * (arm_length - min_arm_length)
    }

    fun calculateCog(arm_length: Double): Double {
        return min_arm_cog + (max_arm_cog - min_arm_cog) / (max_arm_length - min_arm_length) * (arm_length - min_arm_length)
    }
    fun getCurrentDrawAmps(): Double {
        return motor_count * currentDraw
    }

    fun getMinArmLength(): Double {
        return min_arm_length
    }
    fun getMaxArmLength(): Double {
        return max_arm_length
    }

    fun angleAtJoint() : Double {
        return joint_theta
    }
}