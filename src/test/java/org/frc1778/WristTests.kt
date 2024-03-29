package org.frc1778

import org.frc1778.simulation.WristSim
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.junit.jupiter.api.Test

class WristTests {
    @Test
    fun printCoefficients() {
        var wristSim = WristSim(0.radians)
        System.out.printf("Ka: %s\n", wristSim.Ka())
        System.out.printf("Kv: %s\n", wristSim.Kv())
        System.out.printf("Ks: %s\n", wristSim.Ks())
        println(1 / Constants.ShooterConstants.ANGLE_MOTOR_GEAR_REDUCTION)
    }

    /*
    @Test
    fun moveWrist() {
        runCommand()
    }

    fun runCommand() {
        var wristSim = WristSim(initialWristPosition = (0).degrees)
        var encoderSim = DutyCycleEncoderSim(Manipulator.encoder.dutyCycleEncoder)
        encoderSim.setDistance(Constants.ArmConstants.DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(wristSim.wristPosition).value)
        //Assertions.assertEquals(0.degrees.value, Manipulator.encoder.absolutePosition.value, 0.01)
        Arm.zeroed = true
        Manipulator.initialize()
        SimHooks.pauseTiming()
        var cmd = ManipulatorAngleCommand(70.degrees)
        cmd.initialize()
        var i = 0
        var dt = 0.002
        var sum = 0.0
        Manipulator.periodic()
        while (i++ < 10000 ) {
            if (sum >= 0.02) {
                Manipulator.periodic()
                if(!cmd.isFinished) {
                    cmd.execute()
                }
                sum = 0.0
            } else {
                sum += dt
            }
            wristSim.setInput(Manipulator.angleMotor.canSparkMax.appliedOutput.volts)


            wristSim.update(dt.seconds, 0.degrees)
            encoderSim.setDistance(Constants.ArmConstants.DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(wristSim.wristPosition).value)

            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    wristSim.getCurrentDrawAmps()
                )
            )
            SimHooks.stepTiming(dt)

            if(cmd.isFinished) {
                cmd.end(false)
            }
        }
    }

     */
}
