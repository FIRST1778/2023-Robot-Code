/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.frc1778.lib

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem

abstract class FalconTimedRobot {

    enum class Mode {
        NONE,
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST,
        SIMULATION
    }

    var currentMode = Mode.NONE
    private set

    private val sensorlessReadySystems = arrayListOf<SensorlessCompatibleSubsystem>()
    var sensorlessModeActive = false
    protected set

    protected val wrappedValue = WpiTimedRobot()

    protected inner class WpiTimedRobot : TimedRobot() {

        private val kLanguage_Kotlin = 6

        init {
            HAL.report(FRCNetComm.tResourceType.kResourceType_Language, kLanguage_Kotlin)
        }

        override fun robotInit() {
            currentMode = FalconTimedRobot.Mode.NONE
            this@FalconTimedRobot.robotInit()
                    FalconSubsystemHandler.lateInit()
            LiveWindow.disableAllTelemetry()
        }

        override fun autonomousInit() {
            currentMode = FalconTimedRobot.Mode.AUTONOMOUS
            this@FalconTimedRobot.autonomousInit()
                    FalconSubsystemHandler.autoReset()
        }

        override fun teleopInit() {
            currentMode = FalconTimedRobot.Mode.TELEOP
            this@FalconTimedRobot.teleopInit()
                    FalconSubsystemHandler.teleopReset()
        }

        override fun disabledInit() {
            currentMode = FalconTimedRobot.Mode.DISABLED
            this@FalconTimedRobot.disabledInit()
                    FalconSubsystemHandler.setNeutral()
        }

        override fun testInit() {
            currentMode = FalconTimedRobot.Mode.TEST
            this@FalconTimedRobot.testInit()
        }

        override fun robotPeriodic() {
            this@FalconTimedRobot.robotPeriodic()
                    CommandScheduler.getInstance().run()
        }

        override fun autonomousPeriodic() {
            this@FalconTimedRobot.autonomousPeriodic()
        }

        override fun teleopPeriodic() {
            this@FalconTimedRobot.teleopPeriodic()
        }

        override fun disabledPeriodic() {
            this@FalconTimedRobot.disabledPeriodic()
        }
        override fun simulationInit() {
            currentMode = FalconTimedRobot.Mode.SIMULATION
            this@FalconTimedRobot.simulationInit()
        }

        override fun simulationPeriodic() {
            this@FalconTimedRobot.simulationPeriodic()
        }
    }

    protected open fun robotInit() {}
    protected open fun autonomousInit() {}
    protected open fun teleopInit() {}
    protected open fun disabledInit() {}
    protected open fun testInit() {}

    protected open fun robotPeriodic() {}
    protected open fun autonomousPeriodic() {}
    protected open fun teleopPeriodic() {}
    protected open fun disabledPeriodic() {}

    protected open fun simulationPeriodic() {}
    protected open fun simulationInit() {}
    protected fun getSubsystemChecks(): Command {
        return FalconSubsystemHandler.testCommand
    }

    open operator fun FalconSubsystem.unaryPlus() {
        FalconSubsystemHandler.add(this)
        if (this is SensorlessCompatibleSubsystem) {
            sensorlessReadySystems.add(this)
        }
    }

    fun disableClosedLoopControl() {
        sensorlessReadySystems.forEach { it.disableClosedLoopControl() }
        sensorlessModeActive = true
    }

    fun enableClosedLoopControl() {
        sensorlessReadySystems.forEach { it.enableClosedLoopControl() }
        sensorlessModeActive = false
    }

    fun start() {
        RobotBase.startRobot { wrappedValue }
    }
}
