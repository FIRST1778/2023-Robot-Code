package org.frc1778.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.lib.commands.FalconSubsystem
import org.littletonrobotics.junction.Logger

object Vision: FalconSubsystem() {
    lateinit var io: VisionIO
    val inputs = VisionInputsAutoLogged()
    var alliance = DriverStation.Alliance.Blue

    override fun autoReset() {
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    }

    override fun teleopReset() {
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Vision", inputs)
        Logger.recordOutput("Vision/Target Poses", *inputs.targetPoses.toTypedArray())
        Logger.recordOutput("Vision/Robot Pose", inputs.robotPose3d)
    }

    fun getRobotPose3d(): Pair<Pose3d, Double>? {
        return io.getRobotPose3d()
    }

    fun withIO(io: VisionIO): Vision {
        this.io = io
        return this
    }
}