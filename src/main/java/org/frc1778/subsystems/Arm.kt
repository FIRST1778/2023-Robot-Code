// TODO safety check: only let Arm move when in the right area.

package org.frc1778.subsystems

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
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians

object Arm : FalconSubsystem() {
    var armJointSim = ArmJointSim(Math.PI/4)

    var encoder = Encoder(1, 2, false, CounterBase.EncodingType.k4X)
    var encoderSim = EncoderSim(encoder)

    var angleMotorMain = PWMSparkMax(2)

    var kA : Double = 0.03516
    var kV : Double = 1.615
    var plant = LinearSystemId.identifyPositionSystem(kV, kA)

    val Ks : Double = 0.3851

    // This is set in the ArmTrapezoidCommand to what the profile wants, so we
    // initialize it to a dud value here to not immediately activate the arm.
    var desiredAngle : Double = Math.toRadians(45.0)

    var angleControlEnabled : Boolean = true

    var observer = KalmanFilter(
            Nat.N2(),
            Nat.N1(),
            plant,
            VecBuilder.fill(0.5, 0.5),
            VecBuilder.fill(0.01),
            0.02
    )
    var controller = LinearQuadraticRegulator(
            plant,
            VecBuilder.fill(2.0, 16.0),
            VecBuilder.fill(24.0),
            0.020
    )
    var loop = LinearSystemLoop(
            plant,
            controller,
            observer,
            12.0,
            0.020
    )

    fun setAngle( angle : SIUnit<Radian>) {
        desiredAngle = angle.value
    }
    fun angleControl(){
        if(angleControlEnabled) {
            loop.setNextR((VecBuilder.fill(desiredAngle, 0.0)))
            loop.correct(VecBuilder.fill(getCurrentAngle().value))
            loop.predict(0.020) // 20 ms

            var nextVoltage = loop.getU(0)
            nextVoltage += Ks * Math.sin(getCurrentAngle().value)
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            angleMotorMain.setVoltage(nextVoltage)
        }else{
            angleMotorMain.setVoltage(0.0)
        }
    }
    fun getCurrentAngle() : SIUnit<Radian>{
        return encoder.distance.radians
    }
    override fun lateInit() {
        encoder.setSamplesToAverage(5)

        encoder.setDistancePerPulse((2 * Math.PI) / 1024)

        encoder.setMinRate(1.0)
    }
    override fun periodic() {
        angleControl()
    }
    override fun simulationPeriodic(){
        armJointSim.setInput(angleMotorMain.get() * RobotController.getBatteryVoltage())

        armJointSim.update(0.02, armJointSim.getMinArmLength())

        encoderSim.distance = armJointSim.angleAtJoint()
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armJointSim.getCurrentDrawAmps()))
    }
}