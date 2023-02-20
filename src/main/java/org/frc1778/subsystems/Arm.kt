// TODO safety check: only let Arm move when in the right area.

package org.frc1778.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import org.frc1778.ArmJointSim
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax // TODO
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.frc1778.ExtensionSim
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.sin

object Arm : FalconSubsystem() {
    var armJointSim = ArmJointSim(Math.toRadians(135.0))
    var extensionSim = ExtensionSim(0.0.meters)

    var armEncoder = Encoder(1, 2, false, CounterBase.EncodingType.k4X)
    var extensionEncoder = Encoder(3, 4, false, CounterBase.EncodingType.k4X)
    var armEncoderSim = EncoderSim(armEncoder)
    var extensionEncoderSim = EncoderSim(extensionEncoder)

    var angleMotorMain = PWMSparkMax(11)
    var extensionMotor = PWMSparkMax(12)


    var kA : Double = 0.03516
    var kV : Double = 1.615
    var plant = LinearSystemId.identifyPositionSystem(kV, kA)

    val Ks : Double = 0.3851

    // This is set in the ArmTrapezoidCommand to what the profile wants, so we
    // initialize it to a dud value here to not immediately activate the arm.
    var desiredAngle : Double = Math.toRadians(135.0)
    var desiredExtensionPosition : SIUnit<Meter> = 0.0.meters

    var angleControlEnabled : Boolean = true
    var extensionControlEnabled : Boolean = true

    var angleObserver = KalmanFilter(
            Nat.N2(),
            Nat.N1(),
            plant,
            VecBuilder.fill(0.5, 0.5),
            VecBuilder.fill(0.01),
            0.02
    )
    var angleController = LinearQuadraticRegulator(
            plant,
            VecBuilder.fill(2.0, 16.0),
            VecBuilder.fill(24.0),
            0.020
    )
    var angleLoop = LinearSystemLoop(
            plant,
            angleController,
            angleObserver,
            12.0,
            0.020
    )



    fun setAngle( angle : SIUnit<Radian>) {
        desiredAngle = angle.value
    }
    fun angleControl(){
        if(angleControlEnabled) {
            angleLoop.setNextR(VecBuilder.fill(desiredAngle, 0.0))
            angleLoop.correct(VecBuilder.fill(getCurrentAngle().value))
            angleLoop.predict(0.020) // 20 ms

            var nextVoltage = angleLoop.getU(0)
            nextVoltage += Ks * sin(getCurrentAngle().value)
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            angleMotorMain.setVoltage(nextVoltage)
        }else{
            angleMotorMain.setVoltage(0.0)
        }
    }
    fun extensionControl(){
        if (extensionControlEnabled){

        }
    }
    fun getCurrentExtensionPosition() : SIUnit<Meter>{
        return extensionEncoder.distance.meters
    }
    fun setExtensionPosition(position : SIUnit<Meter>){
        desiredExtensionPosition = position
    }
    fun getCurrentAngle() : SIUnit<Radian>{
        return armEncoder.distance.radians
    }
    override fun lateInit() {
        armEncoder.setSamplesToAverage(5)

        armEncoder.setDistancePerPulse((2 * Math.PI) / 1024)

        armEncoder.setMinRate(1.0)

        armEncoderSim.distance = armJointSim.angleAtJoint()
    }
    override fun periodic() {
        angleControl()
        extensionMotor.setVoltage(5.0)
    }
    override fun simulationPeriodic() {
        extensionSim.setInput(extensionMotor.get() * RobotController.getBatteryVoltage())
        armJointSim.setInput(angleMotorMain.get() * RobotController.getBatteryVoltage())

        extensionSim.update(0.02, armJointSim.angleAtJoint())
        armJointSim.update(0.02, armJointSim.getMinArmLength())

        extensionEncoderSim.distance = extensionSim.getCurrentArmPosition()
        armEncoderSim.distance = armJointSim.angleAtJoint()

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armJointSim.getCurrentDrawAmps(), extensionSim.getCurrentDrawAmps()))
    }

}