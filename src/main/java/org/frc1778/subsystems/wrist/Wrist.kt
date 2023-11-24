package org.frc1778.subsystems.wrist

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.Level
import org.frc1778.lib.DataLogger
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import kotlin.math.sin

object Wrist : FalconSubsystem() {

    val io = if(LoggedRobot.isReal()) {
        WristIOSparkMax()
    } else {
        WristIOSim()
    }

    val wristInputs = WristInputsAutoLogged()


    //TODO: Update to new weights
    private var feedforwardVoltage = 0.0
    private const val angle_kS: Double = 0.0
    private const val angle_kA: Double = 0.05222222222222221
    private const val angle_kV: Double = 1.071505486997903
    private val anglePlant = LinearSystemId.identifyPositionSystem(angle_kV, angle_kA)

    private var desiredAngleVelocity: Double = 0.0 // rad/s
    private var desiredAngle: SIUnit<Radian> = 0.0.radians

    val angleControlEnabled = true

    var dataLogger = DataLogger("Shooter")



    init {
        dataLogger.add("position", { getCurrentAngle().inDegrees() })
        dataLogger.add("desired position", { desiredAngle.inDegrees() })
        dataLogger.add("desired velocity", { desiredAngleVelocity })
        dataLogger.add("ss voltage", { angleLoop.getU(0) })
        dataLogger.add("ff voltage", { feedforwardVoltage })
    }

    private var scoringLevel: Level = Level.None
    private var nextLevel: Level? = null

    fun setDesiredAngleVelocity(velocity: Double) {
        desiredAngleVelocity = velocity
    }

    fun setDesiredAngle(angle: SIUnit<Radian>) {
        desiredAngle = angle
    }

    fun getCurrentAngle(): SIUnit<Radian> {
        return io.getCurrentAngle()
    }

    fun getDesiredAngleVelocity(): Double {
        return desiredAngleVelocity
    }

    private val angleObserver = KalmanFilter(
        Nat.N2(), Nat.N1(), anglePlant, VecBuilder.fill(0.5, 0.5), VecBuilder.fill(0.01), 0.02
    )
    private val angleController = LinearQuadraticRegulator(
        anglePlant, VecBuilder.fill(0.075, 0.075), VecBuilder.fill(.5), 0.020
    )
    private val angleLoop = LinearSystemLoop(
        anglePlant, angleController, angleObserver, 12.0, 0.020
    )

    //TODO Get angle offset for gravity/feedforward
    val angleOffset: SIUnit<Radian> = 36.0.degrees
    private fun angleControl() {
        @Suppress("UsePropertyAccessSyntax")
        if (angleControlEnabled && ((scoringLevel != Level.None) || (nextLevel != Level.None)) || getCurrentAngle() > 92.degrees) {
            angleLoop.setNextR(VecBuilder.fill(desiredAngle.value, desiredAngleVelocity))
            angleLoop.correct(VecBuilder.fill(getCurrentAngle().value))
            angleLoop.predict(0.020) // 20 ms

            var nextVoltage = angleLoop.getU(0)

            //TODO: See if this is throwing things off
            feedforwardVoltage = angle_kS * sin(getCurrentAngle().value + angleOffset.value)
            nextVoltage += feedforwardVoltage
            if (nextVoltage > 12) {
                nextVoltage = 12.0
            }

            io.setVoltage(nextVoltage.volts)

        } else {
            if (LoggedRobot.isReal()) {
                resetDesiredAngle()
                io.setVoltage(0.0.volts)
            }
        }
    }

    fun setNextLevel(level: Level) {
        nextLevel = level
    }

    fun resetDesiredAngle() {
        desiredAngle = getCurrentAngle()
        desiredAngleVelocity = 0.0
    }

    fun setBrakeMode(brakeMode: Boolean) {
        io.brakeMode = brakeMode
    }

    override fun periodic() {
        try {
            angleControl()
        } catch (e: Exception) {
            DriverStation.reportError("Bad Angle ${getCurrentAngle()}", false)
        }

        io.updateInputs(wristInputs)
        Logger.processInputs("Wrist Inputs",wristInputs)

        Logger.recordOutput(
            "Wrist Pose", Pose3d(
                Translation3d(-0.255, 0.0, 0.3175),
                Rotation3d(0.0, (-(getCurrentAngle()) + 90.0.degrees).value, 0.0)
            )
        )
        Logger.recordOutput(
            "Wrist Angle",
            getCurrentAngle().value
        )
    }



    fun setScoringLevel(level: Level) {
        scoringLevel = level
    }

    fun getScoringLevel(): Level {
        return scoringLevel
    }


    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder!!)
        builder.addStringProperty("Level", { scoringLevel.name }, {})
        builder.addDoubleProperty("Angle", { getCurrentAngle().inDegrees() }, {})
    }

}