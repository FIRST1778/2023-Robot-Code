package org.frc1778.commands

import ArmPosition
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import org.frc1778.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.meters

class ArmRotateAndExtend(val armPosition : ArmPosition) : FalconCommand(Arm){
    companion object {
        const val ANGLE_END_VEL = 0.0     // meter/sec
        const val ANGLE_MAX_VEL = 1.10
        const val ANGLE_MAX_ACCEL = 0.75
        const val EXTENSION_END_VEL = 0.0     // rad/sec
        const val EXTENSION_MAX_VEL = 1.10
        const val EXTENSION_MAX_ACCEL = 0.75
    }

    private var angleProfile: TrapezoidProfile? = null
    private var extensionProfile: TrapezoidProfile? = null

    private var angleTimer = Timer()
    private var extensionTimer = Timer()
    private var extensionTimerStarted = false
    private var angleTimerStarted = false
    private var invalidNumber = false

    override fun initialize() {

        angleTimer.reset()
        angleTimerStarted = false

        extensionTimer.reset()
        extensionTimerStarted = false

        var angleStartPosition: SIUnit<Radian> = Arm.getCurrentAngle()
        var extensionStartPosition : SIUnit<Meter> = Arm.getCurrentExtension()

        val angleConstraints = TrapezoidProfile.Constraints(ANGLE_MAX_VEL, ANGLE_MAX_ACCEL)
        val angleStartState = TrapezoidProfile.State(angleStartPosition.value, Arm.getDesiredAngleVelocity())
        val angleEndState = TrapezoidProfile.State(armPosition.desiredAngle.value, ANGLE_END_VEL)
        angleProfile = TrapezoidProfile(angleConstraints, angleEndState, angleStartState)

        val extensionConstraints = TrapezoidProfile.Constraints(EXTENSION_MAX_VEL, EXTENSION_MAX_ACCEL)
        val extensionStartState = TrapezoidProfile.State(extensionStartPosition.value, Arm.getDesiredExtensionVelocity())
        val extensionEndState = TrapezoidProfile.State(armPosition.desiredExtension.value, EXTENSION_END_VEL)
        extensionProfile = TrapezoidProfile(extensionConstraints, extensionEndState, extensionStartState)
    }

    override fun execute() {
        if(armPosition == ArmPosition.TOP){
            if(!angleTimerStarted){ // Third Level
                angleTimer.start()
            }
            val angleState = angleProfile!!.calculate(angleTimer.get())
            Arm.setDesiredAngleVelocity(angleState.velocity)
            Arm.setDesiredAngle(angleState.position.radians)
            if(Arm.getCurrentAngle() >= 60.0.degrees){
                if(!extensionTimerStarted){
                    extensionTimer.start()
                    extensionTimerStarted = true
                }
                val extensionState = extensionProfile!!.calculate(extensionTimer.get())
                Arm.setDesiredExtensionVelocity(extensionState.velocity)
                Arm.desiredExtension = extensionState.position.meters
            }
        }else if(armPosition == ArmPosition.MIDDLE){ // Second Level
            if(Arm.getCurrentAngle() > armPosition.desiredAngle) {
                if (!extensionTimerStarted) {
                    extensionTimer.start()
                    extensionTimerStarted = true
                }
                val extensionState = extensionProfile!!.calculate(extensionTimer.get())
                Arm.setDesiredExtensionVelocity(extensionState.velocity)
                Arm.desiredExtension = extensionState.position.meters
                if (Arm.getCurrentExtension() < 0.75.meters) {
                    if (!angleTimerStarted) {
                        angleTimer.start()
                    }
                    val angleState = angleProfile!!.calculate(angleTimer.get())
                    Arm.setDesiredAngleVelocity(angleState.velocity)
                    Arm.setDesiredAngle(angleState.position.radians)
                }
            }else if(Arm.getCurrentAngle() < armPosition.desiredAngle) {
                if (!angleTimerStarted) {
                    angleTimer.start()
                }
                val angleState = angleProfile!!.calculate(angleTimer.get())
                Arm.setDesiredAngleVelocity(angleState.velocity)
                Arm.setDesiredAngle(angleState.position.radians)
                if (Arm.getCurrentAngle() >= 70.0.degrees) {
                    if (!extensionTimerStarted) {
                        extensionTimer.start()
                        extensionTimerStarted = true
                    }
                    val extensionState = extensionProfile!!.calculate(extensionTimer.get())
                    Arm.setDesiredExtensionVelocity(extensionState.velocity)
                    Arm.desiredExtension = extensionState.position.meters
                }
            }
        }else if(armPosition == ArmPosition.BOTTOM) {
            if (!extensionTimerStarted) {
                extensionTimer.start()
                extensionTimerStarted = true
            }
            val extensionState = extensionProfile!!.calculate(extensionTimer.get())
            Arm.setDesiredExtensionVelocity(extensionState.velocity)
            Arm.desiredExtension = extensionState.position.meters
            if (Arm.getCurrentExtension() < 0.5.meters) {
                if (!angleTimerStarted) {
                    angleTimer.start()
                }
                val angleState = angleProfile!!.calculate(angleTimer.get())
                Arm.setDesiredAngleVelocity(angleState.velocity)
                Arm.setDesiredAngle(angleState.position.radians)
            }
        }
    }

    override fun isFinished(): Boolean {
        return invalidNumber || (angleProfile!!.isFinished(angleTimer.get()) && extensionProfile!!.isFinished(extensionTimer.get()))
    }

    override fun end(interrupted: Boolean) {
        if(interrupted){
            Arm.setDesiredAngleVelocity(0.0)
            Arm.setDesiredExtensionVelocity(0.0)
        }
        super.end(interrupted)
    }
    override fun cancel() {
        Arm.setDesiredAngleVelocity(0.0)
        Arm.setDesiredExtensionVelocity(0.0)
        super.cancel()
    }
}
