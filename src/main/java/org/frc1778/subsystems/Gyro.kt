package org.frc1778.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.frc1778.Constants
import org.frc1778.Robot
import org.frc1778.lib.AbstractFalconGyro
import org.frc1778.subsystems.drive.Drive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.round

object Gyro: AbstractFalconGyro() {
    // Because the gyro subsystem only exposes read-only members, any number of
    // commands can access it at the same time.  So don't worry about declaring it
    // as a command dependency.

    private val pigeon = Pigeon2(Constants.DriveConstants.pigeonCanID)

    init {
        pigeon.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ, 500)
    }

    override fun yaw() = Math.toRadians(pigeon.yaw)
    override fun pitch() = Math.toRadians(pigeon.pitch)
    override fun roll() = Math.toRadians(pigeon.roll)
    override fun odometryYaw() = MathUtil.angleModulus(Drive.robotPosition.rotation.radians)

    fun direction180(): Double {
        return abs(PI * round(odometryYaw() / PI))
    }

    fun directionTowardsGrid(): Double {
        return when (Robot.alliance) {
            DriverStation.Alliance.Red -> 0.0
            else -> PI
        }
    }

    init {
        Shuffleboard.getTab("Gyro").add("Gyro", this)
    }
}
