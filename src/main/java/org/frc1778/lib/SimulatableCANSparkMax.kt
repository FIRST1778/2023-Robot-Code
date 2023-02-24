package org.frc1778.lib

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController

class SimulatableCANSparkMax(deviceId : Int, motorType: CANSparkMaxLowLevel.MotorType)  : CANSparkMax(deviceId, motorType){
    var voltage = 0.0;
    override fun setVoltage(outputVolts: Double) {
        if(RobotBase.isSimulation()) {
            voltage = outputVolts;
        }else {
            super.setVoltage(outputVolts)
        }
    }

    override fun get(): Double {
        if(RobotBase.isSimulation()) {
            var batteryVoltage = RobotController.getBatteryVoltage()
            if(batteryVoltage == 0.0) {
                return 0.0;
            }else {
                return (voltage / batteryVoltage).coerceIn(-1.0, 1.0)
            }
        }else{
            return super.get()
        }
    }

    override fun getEncoder(): RelativeEncoder {
        if(RobotBase.isSimulation()) {
            return SimulationRelativeEncoder()
        }else{
            return super.getEncoder();
        }
    }
}