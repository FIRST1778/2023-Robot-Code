package org.frc1778.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.frc1778.Constants
import org.ghrobotics.lib.commands.FalconSubsystem

object Gyro: FalconSubsystem(), Sendable {
    // Because the gyro subsystem only exposes read-only members, any number of
    // commands can access it at the same time.  So don't worry about declaring it
    // as a command dependency.

    private val pigeon = Pigeon2(Constants.DriveConstants.pigeonCanID)

    init {
        pigeon.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ, 500)
    }

    fun rawYaw() = Math.toRadians(pigeon.yaw)
    fun odometryYaw() = Drive.robotPosition.rotation.radians
    fun pitch() = Math.toRadians(pigeon.pitch)
    fun roll() = Math.toRadians(pigeon.roll)

    override fun lateInit() {
        // TODO add this to the shuffleboard properly (something like ...)
        // Constants.ShooterConstants.shooterTab.add(
        //    this
        // ).withSize(3, 4)
    }

    fun boardInclination(): Double {
        // Our robot is conceptually a rectangle, and a rectangle is basically a square,
        // and squares are planes.  So think of robot rotation as defining a plane.
        // Our goal is to find the angle between the robot plane and the ground, going in the
        // X direction.
        //
        // A plane is defined uniquely by a point on it and a vector (starting at the point)
        // perpendicular to the plane --- that is, pointing up.  The point will be (0,0,0)
        // and the vector will be (0,0,1).  We rotate by yaw, pitch, and roll to find the
        // actual current value of the vector.
        //
        // To find the angle of the plane along the X axis, we calculate a point on the plane
        // (1, 0, z).  If the rotated vector ends at (a, b, c):
        //     ax + by + cz = 0
        // Substituting and rearranging:
        //     a(1) + b(0) + cz = 0
        //     a + cz = 0
        //     cz = -a
        //     z = -a/c
        val rotation = Rotation3d(roll(), pitch(), odometryYaw())
        val upVector = Translation3d(0.0, 0.0, 1.0).rotateBy(rotation)
        val z: Double = -upVector.x/upVector.z
        return Math.atan(z)
    }

    init {
        Shuffleboard.getTab("Gyro").add(this)
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        super.initSendable(builder)
        builder.addDoubleProperty("Odometry Yaw (deg)", { Math.toDegrees(odometryYaw()) }, {})
        builder.addDoubleProperty("Raw Yaw (deg)", { pigeon.yaw }, {})
        builder.addDoubleProperty("Pitch (deg)", { pigeon.pitch }, {})
        builder.addDoubleProperty("Roll (deg)", { pigeon.roll }, {})
        builder.addDoubleProperty("Inclination (deg)", { Math.toDegrees(boardInclination()) }, {})
    }
}
