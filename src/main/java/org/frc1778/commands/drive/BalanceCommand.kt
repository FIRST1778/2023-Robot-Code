package org.frc1778.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import org.frc1778.Robot
import org.frc1778.subsystems.Drive
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.atan
import kotlin.math.sin

class BalanceCommand: FalconCommand(Drive) {
    companion object {
        const val PROPORTIONAL: Double = 1.95 // (m/sec)/m
        const val INTEGRAL: Double = 0.05
        const val DERIVATIVE: Double = 0.17
        const val ERROR_TOLERANCE: Double = 0.055
        const val ERROR_DERIVATIVE_TOLERANCE: Double = 0.4

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
            val yaw = Drive.robotPosition.rotation.radians
            val pitch = Math.toRadians(Drive.pigeon.pitch)
            val roll = Math.toRadians(Drive.pigeon.roll)
            val rotation = Rotation3d(roll, pitch, yaw)
            val upVector = Translation3d(0.0, 0.0, 1.0).rotateBy(rotation)
            val z: Double = -upVector.x/upVector.z
            return atan(z)
        }
    }

    // Note that the PIDController defaults to being called every 20ms.  So we must
    // calculate a velocity ONCE per tick: not more, not less.
    lateinit var pid: PIDController
    override fun initialize() {
        pid = PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE)
        pid.enableContinuousInput(0.0, 2.0*Math.PI)
        pid.setTolerance(ERROR_TOLERANCE, ERROR_DERIVATIVE_TOLERANCE)
    }

    override fun execute() {
        val inclination = boardInclination()
        val velocity = pid.calculate(sin(inclination), 0.0)
        Drive.swerveDrive(-velocity, 0.0, 0.0, true)
    }

    override fun end(interrupted: Boolean) {
        DriveBrakeCommand().schedule()
    }

    override fun isFinished(): Boolean {
        return pid.atSetpoint()
    }
}