package org.frc1778.subsystems.wrist

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches


class WristIOSim : WristIO {

    val lowerAngleLimit = 90.degrees

    private val armSim = SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNEO(1), 0.092894749025, 20 * (64.0 / 24.0)
        ), DCMotor.getNEO(1), 20 * (64.0 / 24.0), 12.068.inches.value, lowerAngleLimit.value, 275.degrees.value, true, 90.degrees.value
    )

    private var voltage = 0.0.volts
    override fun updateInputs(inputs: WristInputs) {
        armSim.update(0.02)
        if(armSim.hasHitLowerLimit()) {
            armSim.setState(lowerAngleLimit.value, armSim.velocityRadPerSec)
        }
        inputs.angleMotorVoltage = voltage
        inputs.currentAngle = armSim.angleRads.radians
        inputs.velocity = SIUnit(armSim.velocityRadPerSec)
        inputs.angleMotorCurrent = armSim.currentDrawAmps.amps

    }

    override fun getCurrentAngle(): SIUnit<Radian> {
        return armSim.angleRads.radians
    }

    override fun setVoltage(voltage: SIUnit<Volt>) {
        this.voltage = voltage
        armSim.setInputVoltage(voltage.value)
    }

    override fun resetPosition(angle: SIUnit<Radian>) {
       armSim.setState(angle.value, 0.0)
    }

    override var brakeMode: Boolean
        get() = false
        set(value) {}
    override val brakeModeSwitch: Boolean
        get() = false
}