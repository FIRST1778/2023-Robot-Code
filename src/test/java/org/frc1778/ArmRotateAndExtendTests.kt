package org.frc1778

class ArmRotateAndExtendTests {

    /*
    @Test
    fun testGoToTop() {
        var armJointSim = ArmJointSim(16.0.degrees.value)
        var extensionSim = ExtensionSim(0.0.mete*s)
        var encoderSim = DutyCycleEncoderSim(Arm.armEncoder.dutyCycleEncoder)
        var limitSwitchSim = DIOSim(Arm.limitSwitch)
        //Arm.initialize()
        encoderSim.setDistance(-DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(armJointSim.angleAtJoint().radians).value)
        var a = Arm.getCurrentAngle()
        Assertions.assertEquals(16.degrees.value, Arm.getCurrentAngle().inRadians(), 0.01)
        Arm.zeroed = true
        Arm.resetDesiredAngle()
        Arm.resetDesiredExtension()
        Arm.angleControlEnabled = true
        Arm.extensionControlEnabled = true
        var cmd = ArmRotateAndExtend(ArmPosition.TOP)
        cmd.initialize()
        var i = 0
        var dt = 0.002
        var sum = 0.0
        SimHooks.pauseTiming()
        Arm.periodic()
        while (i++ < 10000 && !cmd.isFinished) {
            if (sum >= 0.02) {
                Arm.periodic()
                cmd.execute()
                sum = 0.0
            } else {
                sum += dt
            }
            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)
            extensionSim.setInput(Arm.extensionMotor.canSparkMax.appliedOutput)


            armJointSim.update(dt, armJointSim.getMinArmLength())
            encoderSim.setDistance(-DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(armJointSim.angleAtJoint().radians).value)

            extensionSim.update(dt, Arm.getCurrentAngle().inRadians(), Arm.extensionEncoder, limitSwitchSim)

            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    armJointSim.getCurrentDrawAmps(),
                    extensionSim.getCurrentDrawAmps()
                )
            )
            SimHooks.stepTiming(dt)

            if(cmd.isFinished) {
                cmd.end(false)
            }
        }
    }

    @Test
    fun testGoToMiddleFromTop() {
        runCommand(startingAngle=110.degrees, startingExtension=1.meters, desiredPosition= ArmPosition.MIDDLE)
    }

    @Test
    fun testGoToMiddleFromBottom() {
        runCommand(startingAngle=16.degrees, startingExtension=0.1.meters, desiredPosition= ArmPosition.MIDDLE)
    }

    @Test
    fun testGoToBottomFromTop() {
        runCommand(startingAngle=110.degrees, startingExtension=1.meters, desiredPosition= ArmPosition.BOTTOM)
    }

    fun runCommand(startingAngle: SIUnit<Radian>, startingExtension: SIUnit<Meter>, desiredPosition: ArmPosition) {
        var armJointSim = ArmJointSim(startingAngle.value)
        var extensionSim = ExtensionSim(startingExtension)
        Arm.extensionEncoder.resetPosition(startingExtension)
        var encoderSim = DutyCycleEncoderSim(Arm.armEncoder.dutyCycleEncoder)
        var limitSwitchSim = DIOSim(Arm.limitSwitch)
        encoderSim.setDistance(-DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(armJointSim.angleAtJoint().radians).value)
        Arm.zeroed = true
        Arm.resetDesiredAngle()
        Arm.resetDesiredExtension()
        Arm.angleControlEnabled = true
        Arm.extensionControlEnabled = true
        var cmd = ArmRotateAndExtend(desiredPosition)
        cmd.initialize()
        var i = 0
        var dt = 0.002
        var sum = 0.0
        SimHooks.pauseTiming()
        Arm.periodic()
        while (i++ < 10000 && !cmd.isFinished) {
            if (sum >= 0.02) {
                Arm.periodic()
                cmd.execute()
                sum = 0.0
            } else {
                sum += dt
            }
            armJointSim.setInput(Arm.angleMotorMain.canSparkMax.appliedOutput)
            extensionSim.setInput(Arm.extensionMotor.canSparkMax.appliedOutput)


            armJointSim.update(dt, armJointSim.getMinArmLength())
            encoderSim.setDistance(-DUTY_CYCLE_ABSOLUTE_ENCODER_UNIT_MODEL.toNativeUnitPosition(armJointSim.angleAtJoint().radians).value)

            extensionSim.update(dt, Arm.getCurrentAngle().inRadians(), Arm.extensionEncoder, limitSwitchSim)

            RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    armJointSim.getCurrentDrawAmps(),
                    extensionSim.getCurrentDrawAmps()
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
