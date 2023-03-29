package org.frc1778.lib

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController

class SimulatableCANSparkMax(deviceId: Int, motorType: MotorType) :
    CANSparkMax(deviceId, motorType) {
    var simulationVoltage = 0.0
    var simulationEncoder = SimulationRelativeEncoder()
    override fun setVoltage(outputVolts: Double) {
        if (RobotBase.isSimulation()) {
            simulationVoltage = outputVolts
        } else {
            super.setVoltage(outputVolts)
        }
    }

    override fun get(): Double {
        if (RobotBase.isSimulation()) {
            var batteryVoltage = RobotController.getBatteryVoltage()
            if (batteryVoltage == 0.0) {
                return 0.0
            } else {
                return (simulationVoltage / batteryVoltage).coerceIn(-1.0, 1.0)
            }
        } else {
            return super.get()
        }
    }

    override fun getEncoder(): RelativeEncoder {
        if (RobotBase.isSimulation()) {
            return simulationEncoder
        } else {
            return super.getEncoder()
        }
    }
}