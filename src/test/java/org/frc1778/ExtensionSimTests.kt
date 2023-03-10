package org.frc1778

import org.frc1778.commands.ArmAngleCommand
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.junit.jupiter.api.Test
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.junit.jupiter.api.Assertions

class ExtensionSimTests {
    @Test
    fun aksdflksdfj() {
        Arm.angleMotorMain.setVoltage(4.volts)
        Assertions.assertEquals(4.0, Arm.angleMotorMain.canSparkMax.appliedOutput)
    }
    @Test
    fun balls(){
        var armJointSim = ArmJointSim(0.0)
        var encoderSim = DutyCycleEncoderSim(Arm.armEncoder)
        Arm.zeroed = true
        Arm.angleControlEnabled = true
        var cmd = ArmAngleCommand(90.degrees)
        cmd.initialize()
        var i = 0
        var dt = 0.002
        var sum = 0.0
        SimHooks.pauseTiming()
        Arm.periodic()
        while(i++ < 1000) {
            if(sum >= 0.02) {
                Arm.periodic()
                cmd.execute()
                sum = 0.0
            }else {
                sum += dt
            }
            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)


            armJointSim.update(dt, armJointSim.getMinArmLength())


            Arm.setDesiredAngleVelocity()
            encoderSim.setDistance(armJointSim.angleAtJoint())

            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    armJointSim.getCurrentDrawAmps()
                )
            )
            SimHooks.stepTiming(dt)
        }
        armJointSim = ArmJointSim(Arm.getCurrentAngle().value)
        Arm.zeroed = true
        Arm.angleControlEnabled = true
        var cmd2 = ArmAngleCommand(135.degrees)
        cmd2.initialize()
        dt = 0.002
        sum = 0.0
        SimHooks.pauseTiming()
        Arm.periodic()
        while(i++ < 2000) {
            if(sum >= 0.02) {
                Arm.periodic()
                cmd2.execute()
                sum = 0.0
            }else {
                sum += dt
            }
            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)


            armJointSim.update(dt, armJointSim.getMinArmLength())


            var encoderDistance = armJointSim.angleAtJoint()
            encoderSim.setDistance(armJointSim.angleAtJoint())


            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    armJointSim.getCurrentDrawAmps()
                )
            )
            SimHooks.stepTiming(dt)
        }
        armJointSim = ArmJointSim(Arm.getCurrentAngle().value)
        Arm.zeroed = true
        Arm.angleControlEnabled = true
        var cmd3 = ArmAngleCommand(40.degrees)
        cmd3.initialize()
        dt = 0.002
        sum = 0.0
        SimHooks.pauseTiming()
        Arm.periodic()
        while(i++ < 10000) {
            if(sum >= 0.02) {
                Arm.periodic()
                cmd3.execute()
                sum = 0.0
            }else {
                sum += dt
            }
            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)


            armJointSim.update(dt, armJointSim.getMinArmLength())


            var encoderDistance = armJointSim.angleAtJoint()
            encoderSim.setDistance(armJointSim.angleAtJoint())


            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    armJointSim.getCurrentDrawAmps()
                )
            )
            SimHooks.stepTiming(dt)
        }
    }

//    @Test
//    fun ballsOnWalls(){
//
//        var armJointSim = ArmJointSim(0.0)
//        Arm.zeroed = true
//        Arm.angleControlEnabled = true
//        Arm.setAngle(135.degrees)
//        var i = 0
//        var dt = 0.002
//        var sum = 0.0
//        SimHooks.pauseTiming()
//        Arm.periodic()
//        while(i++ < 10000) {
//            if(sum >= 0.02) {
//                Arm.periodic()
//                sum = 0.0
//            }else {
//                sum += dt
//            }
//            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)
//
//
//            armJointSim.update(dt, armJointSim.getMinArmLength())
//
//
//            var encoderDistance = armJointSim.angleAtJoint()
//            encoderSim.setDistance(armJointSim.angleAtJoint())
//
//
//            RoboRioSim.setVInVoltage(
//                BatterySim.calculateDefaultBatteryLoadedVoltage(
//                    armJointSim.getCurrentDrawAmps()
//                )
//            )
//            SimHooks.stepTiming(dt)
//        }
//
//    }
}